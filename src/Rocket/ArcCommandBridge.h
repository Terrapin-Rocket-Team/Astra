#ifndef ASTRA_ROCKET_ARC_COMMAND_BRIDGE_H
#define ASTRA_ROCKET_ARC_COMMAND_BRIDGE_H

#include <Arduino.h>
#include <stdint.h>

#include <arc_protocol.h>
#include <arc_reliable.h>

namespace astra_rocket
{
    class AstraRocket;

    static constexpr uint8_t ASTRA_ROCKET_ARC_GET_STATUS = 0x20;
    static constexpr uint8_t ASTRA_ROCKET_ARC_TEXT_COMMAND = 0x21;
    static constexpr uint8_t ASTRA_ROCKET_ARC_STATUS_REPORT = 0x40;
    static constexpr uint8_t ASTRA_ROCKET_ARC_TEXT_RESPONSE = 0x41;
    static constexpr uint8_t ASTRA_ROCKET_ARC_ERROR = 0x7F;

    class ArcCommandBridge
    {
    public:
        ArcCommandBridge(uint8_t addr, Stream *stream, AstraRocket *rocket = nullptr);

        void setRocket(AstraRocket *rocket);
        void begin(uint8_t session = 1, uint16_t firstSeq = 0);
        void update();

        uint8_t address() const { return myAddr; }
        size_t receivedFrameCount() const { return receivedFrames; }
        size_t deliveredFrameCount() const { return deliveredFrames; }
        size_t parseErrorCount() const { return parseErrors; }

    private:
        uint8_t myAddr;
        uint8_t session;
        Stream *stream;
        AstraRocket *rocket;
        arc_reliable_t reliable;
        bool reliableReady;
        uint8_t encodedRx[ARC_MAX_ENCODED_SIZE];
        size_t encodedRxLen;
        size_t receivedFrames;
        size_t deliveredFrames;
        size_t parseErrors;

        void processEncodedFrame();
        void handleDeliveredFrame(const arc_frame_t *frame);
        void handleTextCommand(const arc_frame_t *frame);
        void sendStatusReport(uint8_t dst);
        void sendTextResponse(uint8_t dst, const char *text);
        void sendError(uint8_t dst, const char *text);
        void sendFrame(uint8_t dst, uint8_t family, uint8_t type,
                       const uint8_t *payload, size_t payloadLen, bool reliable);
        void writeFrame(const arc_frame_t *frame);

        static void reliableSend(void *user, const arc_frame_t *frame);
        static void reliableDeliver(void *user, const arc_frame_t *frame);
        static void reliableFail(void *user, const arc_frame_t *frame);
    };

} // namespace astra_rocket

#endif // ASTRA_ROCKET_ARC_COMMAND_BRIDGE_H
