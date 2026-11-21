#pragma once

// Called from Modules.cpp through upstream's optional-modules hook (bin/optional-modules.py) in every env.
void setupButtonMessage();

#ifdef BUTTON_MESSAGE
#include "ButtonMessageLogic.h"
#include "Observer.h"
#include "SinglePortModule.h"
#include "concurrency/OSThread.h"
#include "mesh/generated/meshtastic/cannedmessages.pb.h"

class ButtonThread;

#ifndef BUTTON_MESSAGE_RETRY_MS
#define BUTTON_MESSAGE_RETRY_MS 120000 // critical ('!') broadcasts retry at this interval until acknowledged
#endif
#ifndef BUTTON_MESSAGE_MAX_ATTEMPTS
#define BUTTON_MESSAGE_MAX_ATTEMPTS 60
#endif
#ifndef BUTTON_MESSAGE_ACK_TIMEOUT_MS
#define BUTTON_MESSAGE_ACK_TIMEOUT_MS 60000 // normal messages: fail tone if the mesh has not acknowledged the broadcast
#endif
#ifndef BUTTON_MESSAGE_BOOT_GUARD_MS
#define BUTTON_MESSAGE_BOOT_GUARD_MS 10000 // ignore presses this long after boot (the power-on press); 0 after a button wake
#endif
#ifndef BUTTON_MESSAGE_CLICK_MS
#define BUTTON_MESSAGE_CLICK_MS 250 // max gap between presses of one gesture (stock value); raise if multi-press is fiddly
#endif
#ifndef BUTTON_MESSAGE_MORSE_UNIT_MS
#define BUTTON_MESSAGE_MORSE_UNIT_MS 80 // one morse unit: dot length, gap between elements
#endif

/**
 * Canned messages for screenless devices. N presses (N >= 2) broadcasts canned message N-1 on the first channel.
 * The buzzer signals a mesh acknowledgement. A message starting with '!' is retried until acknowledged or exhausted.
 * Press counts without a message keep the stock behavior. Configured through the stock canned-message settings:
 * canned_message.enabled, canned_message.send_bell and the canned message list.
 */
class ButtonMessageModule : public SinglePortModule, private concurrency::OSThread
{
  public:
    ButtonMessageModule();
    bool queueClicks(const ButtonThread *button, unsigned clicks);
    AdminMessageHandleResult handleAdminMessageForModule(const meshtastic_MeshPacket &mp, meshtastic_AdminMessage *request,
                                                         meshtastic_AdminMessage *response) override;

  protected:
    bool wantPacket(const meshtastic_MeshPacket *p) override;
    ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;
    int32_t runOnce() override;

  private:
    void sayReady();
    void perform(bm::Action a);
    bool sendToFirstChannel();
    void loadMessages();
    bool saveMessages();
    // Power saving: hold off deep sleep (radio off) while a delivery is unresolved
    int preflightSleepCb(void *) { return pendingClicks || logic.state != bm::State::IDLE; }

    bm::Logic logic;
    CallbackObserver<ButtonMessageModule, void *> preflightSleepObserver =
        CallbackObserver<ButtonMessageModule, void *>(this, &ButtonMessageModule::preflightSleepCb);
    meshtastic_CannedMessageModuleConfig messages = meshtastic_CannedMessageModuleConfig_init_zero;
    char current[sizeof(messages.messages)] = {0}; // text of the message in flight, for resends
    unsigned pendingClicks = 0;
    bool pendingCritical = false;
    bool readyTone = false; // button wake: play '.' once the main loop cycles
    uint8_t readyPass = 0;
    PacketId pendingPacketId = 0;
    bm::Event pendingEvent = bm::Event::NONE;
};

extern ButtonMessageModule *buttonMessageModule;
#endif
