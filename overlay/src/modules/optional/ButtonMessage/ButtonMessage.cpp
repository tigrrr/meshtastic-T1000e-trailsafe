#include "ButtonMessage.h"

#ifdef BUTTON_MESSAGE
#include "Channels.h"
#include "FSCommon.h"
#include "GPS.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "SPILock.h"
#include "buzz.h"
#include "input/ButtonThread.h"
#include "sleep.h"
#include <algorithm>

static const char *messagesFile = "/prefs/cannedConf.proto"; // same file and proto as the stock CannedMessageModule

// playTones() and ToneDuration live in buzz.cpp without a header entry; mirrored here verbatim.
struct ToneDuration {
    int frequency_khz;
    int duration_ms;
};
void playTones(const ToneDuration *tone_durations, int size);

// Morse-ish tones: B3 like the stock beep; a 1 Hz note is a rest (nRF52 tone() ignores anything under 20 Hz).
// playTones already waits 1.3x each note, so a 1-unit rest after a dot or dash gives a clear gap.
static const int U = BUTTON_MESSAGE_MORSE_UNIT_MS, B3 = 247, REST = 1;
static const ToneDuration SENT[] = {{B3, U}, {REST, U}, {B3, U}};                                // ..
static const ToneDuration SENT_CRITICAL[] = {{B3, 3 * U}, {REST, U}, {B3, 3 * U}};               // --
static const ToneDuration ACKED[] = {{B3, 3 * U}, {REST, U}, {B3, 3 * U}, {REST, U}, {B3, U}};   // --. (G, good)
static const ToneDuration FAILED[] = {{B3, 7 * U}, {REST, 4 * U}, {B3, 7 * U}}; // two 7-unit tones, ~7 units apart
static const ToneDuration READY[] = {{B3, U}};                                   // . (woken by the button, listening)
#define PLAY(m) playTones(m, sizeof(m) / sizeof(m[0]))

ButtonMessageModule *buttonMessageModule;

// A press ended the tracker's power-saving sleep: there is no power-on press to guard against, so listen at once.
static bool wokeByButton()
{
#ifdef ARCH_NRF52
#ifdef BUTTON_MESSAGE_FAKE_WAKE // bench builds: a timed wake behaves like a button wake
    return trackerWakeCause != TRACKER_WAKE_NONE;
#endif
    return trackerWakeCause == TRACKER_WAKE_BUTTON;
#else
    return false;
#endif
}

void setupButtonMessage()
{
    buttonMessageModule = new ButtonMessageModule();
}

ButtonMessageModule::ButtonMessageModule()
    : SinglePortModule("buttonmsg", meshtastic_PortNum_TEXT_MESSAGE_APP), OSThread("ButtonMessage"),
      logic(wokeByButton() ? 0 : BUTTON_MESSAGE_BOOT_GUARD_MS, BUTTON_MESSAGE_ACK_TIMEOUT_MS, BUTTON_MESSAGE_RETRY_MS,
            BUTTON_MESSAGE_MAX_ATTEMPTS)
{
    loadMessages();
    preflightSleepObserver.observe(&preflightSleep);
    if (moduleConfig.canned_message.enabled) {
        LOG_INFO("ButtonMessage enabled, canned list %s", messages.messages[0] ? "loaded" : "empty");
    } else {
        LOG_INFO("ButtonMessage disabled (canned_message.enabled is false)");
        disable();
        return;
    }
    readyTone = wokeByButton();
}

// Button wake: say that the presses count from here. Presses before this point sat unprocessed in the button state
// machine (the first pass of the main loop blocks for seconds on the GPS probe and the Bluetooth init); drop them so
// they cannot merge with the ones that follow the tone.
void ButtonMessageModule::sayReady()
{
    readyTone = false;
    UserButtonThread->resetClicks();
    LOG_INFO("ButtonMessage: woken by the button, listening");
    PLAY(READY);
#ifdef BUTTON_MESSAGE_FAKE_PRESSES // bench builds: emulate this many presses right after the tone
    LOG_INFO("ButtonMessage: emulating %u presses", (unsigned)BUTTON_MESSAGE_FAKE_PRESSES);
    queueClicks(UserButtonThread, BUTTON_MESSAGE_FAKE_PRESSES);
#endif
}

void ButtonMessageModule::loadMessages()
{
    if (nodeDB->loadProto(messagesFile, meshtastic_CannedMessageModuleConfig_size, sizeof(messages),
                          &meshtastic_CannedMessageModuleConfig_msg, &messages) != LoadFileResult::LOAD_SUCCESS)
        messages.messages[0] = 0; // no list: every press count keeps its stock behavior
}

bool ButtonMessageModule::saveMessages()
{
#ifdef FSCom
    spiLock->lock();
    FSCom.mkdir("/prefs");
    spiLock->unlock();
#endif
    return nodeDB->saveProto(messagesFile, meshtastic_CannedMessageModuleConfig_size, &meshtastic_CannedMessageModuleConfig_msg,
                             &messages);
}

// --- button ---------------------------------------------------------------------------------------------------------

bool ButtonMessageModule::queueClicks(const ButtonThread *button, unsigned clicks)
{
    if (button != UserButtonThread || clicks < 2 || !moduleConfig.canned_message.enabled)
        return false;
    if (readyTone || !logic.acceptsClicks(millis()))
        return true;
    if (!bm::messageAt(messages.messages, clicks - 2, current, sizeof(current), &pendingCritical))
        return false;

    pendingClicks = clicks;
    enabled = true;
    setIntervalFromNow(0);
    return true;
}

// --- thread ---------------------------------------------------------------------------------------------------------

int32_t ButtonMessageModule::runOnce()
{
    if (!moduleConfig.canned_message.enabled) {
        pendingClicks = 0;
        pendingPacketId = 0;
        pendingEvent = bm::Event::NONE;
        logic.state = bm::State::IDLE;
        return disable();
    }
    if (readyTone && UserButtonThread) {
        // Not on the first pass of the main loop: it still has multi-second one-off work ahead of it (GPS probe,
        // Bluetooth init) during which the button thread is not serviced. The second pass means the loop cycles.
        if (++readyPass < 2)
            return 1;
        sayReady();
    }
    uint32_t now = millis();

    const unsigned clicks = pendingClicks;
    pendingClicks = 0;
    if (clicks) {
        LOG_INFO("ButtonMessage: sending canned message for %u presses", clicks);
        perform(logic.onMessage(pendingCritical, now));
    }

    const bm::Event event = pendingEvent;
    pendingEvent = bm::Event::NONE;
    perform(logic.onEvent(event));

    perform(logic.onTick(now));

    uint32_t wake = logic.nextWakeMs(now);
    return wake > 60000 ? 60000 : (int32_t)wake;
}

void ButtonMessageModule::perform(bm::Action a)
{
    switch (a) {
    case bm::Action::NONE:
        break;
    case bm::Action::SEND:
    case bm::Action::RESEND:
#if !MESHTASTIC_EXCLUDE_GPS
        if (logic.critical && gps && config.position.gps_mode == meshtastic_Config_PositionConfig_GpsMode_DISABLED)
            gps->toggleGpsMode(); // a critical message should be able to carry a position
#endif
        if (sendToFirstChannel()) {
            if (logic.critical)
                PLAY(SENT_CRITICAL);
            else
                PLAY(SENT);
            service->refreshLocalMeshNode();
            service->trySendPosition(NODENUM_BROADCAST, false); // stock position broadcast on the configured position channel
            break;
        }
        logic.onEvent(bm::Event::NAK);
        [[fallthrough]];
    case bm::Action::TONE_FAIL:
        if (logic.state == bm::State::IDLE)
            pendingPacketId = 0;
        PLAY(FAILED);
        break;
    case bm::Action::TONE_ACK:
        pendingPacketId = 0;
        PLAY(ACKED);
        break;
    }
}

bool ButtonMessageModule::sendToFirstChannel()
{
    pendingPacketId = 0;
    pendingEvent = bm::Event::NONE;

    constexpr ChannelIndex firstChannel = 0;
    if (channels.getByIndex(firstChannel).role == meshtastic_Channel_Role_DISABLED) {
        LOG_WARN("ButtonMessage: first channel is disabled");
        return false;
    }

    auto *p = allocDataPacket();
    if (!p)
        return false;

    p->to = NODENUM_BROADCAST;
    p->channel = firstChannel;
    p->want_ack = true;
    p->decoded.dest = NODENUM_BROADCAST;

    auto &payload = p->decoded.payload;
    size_t len = 0;
    if (moduleConfig.canned_message.send_bell)
        payload.bytes[len++] = 7; // BEL: the receiver's External Notification bell alert
    const size_t textLen = std::min(strlen(current), sizeof(payload.bytes) - len);
    memcpy(payload.bytes + len, current, textLen);
    payload.size = len + textLen;

    pendingPacketId = p->id;
    LOG_INFO("ButtonMessage: broadcast on channel %u id %u", (unsigned)firstChannel, p->id);
    const auto result = service->sendToMesh(p, RX_SRC_LOCAL, true);
    if (result == ERRNO_OK)
        return true;

    // The router can also report the send error synchronously as a NAK.
    pendingEvent = bm::Event::NONE;
    return false;
}

// --- acks -----------------------------------------------------------------------------------------------------------

bool ButtonMessageModule::wantPacket(const meshtastic_MeshPacket *p)
{
    return pendingPacketId && p->decoded.portnum == meshtastic_PortNum_ROUTING_APP && p->decoded.request_id == pendingPacketId;
}

// Routing feedback can arrive inside sendToMesh, so defer tones to runOnce.
ProcessMessage ButtonMessageModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    if (!isToUs(&mp))
        return ProcessMessage::CONTINUE;

    meshtastic_Routing r = meshtastic_Routing_init_default;
    if (!pb_decode_from_bytes(mp.decoded.payload.bytes, mp.decoded.payload.size, &meshtastic_Routing_msg, &r))
        return ProcessMessage::CONTINUE;

    if (r.which_variant != meshtastic_Routing_error_reason_tag)
        return ProcessMessage::CONTINUE;
    if (r.error_reason == meshtastic_Routing_Error_NONE) {
        // The router generates a local ACK when it hears another node rebroadcast our packet.
        pendingEvent = bm::Event::ACK;
        LOG_INFO("ButtonMessage: broadcast ack for id %u", mp.decoded.request_id);
    } else if (pendingEvent != bm::Event::ACK) {
        pendingEvent = bm::Event::NAK;
        LOG_WARN("ButtonMessage: NAK %d for id %u", r.error_reason, mp.decoded.request_id);
    }
    setIntervalFromNow(0);
    return ProcessMessage::CONTINUE; // RoutingModule must still see it to stop the radio's own retransmits
}

// --- configuration --------------------------------------------------------------------------------------------------

// The stock canned-message admin messages (phone app editor, `meshtastic --set-canned-message`), handled here because
// the stock CannedMessageModule is not built for screenless devices.
AdminMessageHandleResult ButtonMessageModule::handleAdminMessageForModule(const meshtastic_MeshPacket &mp,
                                                                          meshtastic_AdminMessage *request,
                                                                          meshtastic_AdminMessage *response)
{
    switch (request->which_payload_variant) {
    case meshtastic_AdminMessage_get_canned_message_module_messages_request_tag:
        if (mp.decoded.want_response) {
            response->which_payload_variant = meshtastic_AdminMessage_get_canned_message_module_messages_response_tag;
            strncpy(response->get_canned_message_module_messages_response, messages.messages,
                    sizeof(response->get_canned_message_module_messages_response));
        }
        return AdminMessageHandleResult::HANDLED_WITH_RESPONSE;

    case meshtastic_AdminMessage_set_canned_message_module_messages_tag: {
        const char *in = request->set_canned_message_module_messages;
        if (*in && strcmp(messages.messages, in) != 0) {
            strncpy(messages.messages, in, sizeof(messages.messages));
            messages.messages[sizeof(messages.messages) - 1] = 0;
            saveMessages();
            LOG_INFO("ButtonMessage: canned messages updated");
        }
        // Saving messages activates the handler now. The module toggle controls whether it is enabled after reboot.
        if (*in && !moduleConfig.canned_message.enabled) {
            moduleConfig.canned_message.enabled = true;
            enabled = true;
            setIntervalFromNow(0);
        }
        return AdminMessageHandleResult::HANDLED;
    }

    default:
        return AdminMessageHandleResult::NOT_HANDLED;
    }
}

#else
void setupButtonMessage() {}
#endif
