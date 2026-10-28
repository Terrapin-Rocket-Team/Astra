#include "ArcCommandBridge.h"

#include <cstring>
#include <cstdio>

#include "AstraRocket.h"
#include "AstraRocketConfig.h"
#include "FlightStage.h"
#include "RocketState.h"

namespace astra_rocket
{
    ArcCommandBridge::ArcCommandBridge(uint8_t addr, Stream *s, AstraRocket *r)
        : myAddr(addr),
          session(1),
          stream(s),
          rocket(r),
          reliableReady(false),
          encodedRxLen(0),
          receivedFrames(0),
          deliveredFrames(0),
          parseErrors(0)
    {
        std::memset(&reliable, 0, sizeof(reliable));
    }

    void ArcCommandBridge::setRocket(AstraRocket *r)
    {
        rocket = r;
    }

    void ArcCommandBridge::begin(uint8_t initialSession, uint16_t firstSeq)
    {
        session = initialSession;
        arc_reliable_init(&reliable,
                          myAddr,
                          session,
                          1000,
                          3,
                          firstSeq,
                          reliableSend,
                          reliableDeliver,
                          reliableFail,
                          this);
        reliableReady = true;
    }

    void ArcCommandBridge::update()
    {
        if (!stream)
            return;
        if (!reliableReady)
            begin(session, 0);

        while (stream->available())
        {
            int c = stream->read();
            if (c < 0)
                break;

            if (c == 0)
            {
                if (encodedRxLen > 0)
                {
                    if (encodedRxLen < sizeof(encodedRx))
                        encodedRx[encodedRxLen++] = 0;
                    processEncodedFrame();
                }
                encodedRxLen = 0;
                continue;
            }

            if (encodedRxLen < sizeof(encodedRx) - 1)
            {
                encodedRx[encodedRxLen++] = static_cast<uint8_t>(c);
            }
            else
            {
                encodedRxLen = 0;
                ++parseErrors;
            }
        }
    }

    void ArcCommandBridge::processEncodedFrame()
    {
        uint8_t decoded[ARC_MAX_FRAME_SIZE];
        int decodedLen = arc_cobs_decode(encodedRx, encodedRxLen, decoded, sizeof(decoded));
        if (decodedLen < 0)
        {
            ++parseErrors;
            return;
        }

        arc_frame_t frame;
        arc_result_t parsed = arc_frame_parse(decoded, static_cast<size_t>(decodedLen), &frame);
        if (parsed != ARC_OK)
        {
            ++parseErrors;
            return;
        }

        ++receivedFrames;
        if (frame.dst == ARC_ADDR_BROADCAST)
        {
            handleDeliveredFrame(&frame);
            return;
        }

        arc_reliable_receive(&reliable, &frame);
    }

    void ArcCommandBridge::handleDeliveredFrame(const arc_frame_t *frame)
    {
        if (!frame)
            return;

        ++deliveredFrames;
        if (frame->family == ARC_FAMILY_NETMGMT)
        {
            // Reliable NETMGMT frames are answered by arc_reliable_receive().
            return;
        }

        if (frame->family != ARC_FAMILY_FC_COORD)
            return;

        if (frame->type == ASTRA_ROCKET_ARC_GET_STATUS)
        {
            sendStatusReport(frame->src);
            return;
        }
        if (frame->type == ASTRA_ROCKET_ARC_TEXT_COMMAND)
        {
            handleTextCommand(frame);
            return;
        }

        sendError(frame->src, "ERR unsupported-fc-coord-command");
    }

    void ArcCommandBridge::handleTextCommand(const arc_frame_t *frame)
    {
        char command[ARC_MAX_PAYLOAD_SIZE + 1];
        size_t len = frame->payload_len;
        if (len > ARC_MAX_PAYLOAD_SIZE)
            len = ARC_MAX_PAYLOAD_SIZE;
        if (len > 0 && frame->payload)
            std::memcpy(command, frame->payload, len);
        command[len] = '\0';

        char *payload = command;
        while (*payload == ' ')
            ++payload;

        if (std::strncmp(payload, "PING", 4) == 0 &&
            (payload[4] == '\0' || payload[4] == ' '))
        {
            const char *name = "AstraRocket";

            char response[64];
            std::snprintf(response, sizeof(response), "PONG %s", name);
            sendTextResponse(frame->src, response);
            return;
        }

        if (std::strcmp(payload, "STATUS") == 0)
        {
            sendStatusReport(frame->src);
            return;
        }

        sendError(frame->src, "ERR unknown-text-command");
    }

    void ArcCommandBridge::sendStatusReport(uint8_t dst)
    {
        const char *name = "AstraRocket";
        const char *stage = "UNKNOWN";
        int ready = 0;
        long altCm = 0;

        if (rocket)
        {
            ready = rocket->isReady() ? 1 : 0;
            RocketState *state = rocket->getRocketState();
            if (state)
            {
                stage = flightStageToString(state->getFlightStage());
                altCm = static_cast<long>(state->getAltitudeAGL() * 100.0);
            }
        }

        char payload[128];
        std::snprintf(payload,
                      sizeof(payload),
                      "name=%s;ready=%d;stage=%s;alt_cm=%ld",
                      name,
                      ready,
                      stage,
                      altCm);
        sendFrame(dst,
                  ARC_FAMILY_FC_COORD,
                  ASTRA_ROCKET_ARC_STATUS_REPORT,
                  reinterpret_cast<const uint8_t *>(payload),
                  std::strlen(payload),
                  false);
    }

    void ArcCommandBridge::sendTextResponse(uint8_t dst, const char *text)
    {
        if (!text)
            text = "";
        sendFrame(dst,
                  ARC_FAMILY_FC_COORD,
                  ASTRA_ROCKET_ARC_TEXT_RESPONSE,
                  reinterpret_cast<const uint8_t *>(text),
                  std::strlen(text),
                  false);
    }

    void ArcCommandBridge::sendError(uint8_t dst, const char *text)
    {
        if (!text)
            text = "ERR";
        sendFrame(dst,
                  ARC_FAMILY_FC_COORD,
                  ASTRA_ROCKET_ARC_ERROR,
                  reinterpret_cast<const uint8_t *>(text),
                  std::strlen(text),
                  false);
    }

    void ArcCommandBridge::sendFrame(uint8_t dst, uint8_t family, uint8_t type,
                                     const uint8_t *payload, size_t payloadLen, bool wantsReliable)
    {
        if (!reliableReady)
            begin(session, 0);

        uint16_t ignoredSeq = 0;
        arc_reliable_send(&reliable,
                          dst,
                          family,
                          type,
                          0,
                          wantsReliable,
                          payload,
                          payloadLen,
                          millis(),
                          &ignoredSeq);
    }

    void ArcCommandBridge::writeFrame(const arc_frame_t *frame)
    {
        if (!stream || !frame)
            return;

        uint8_t raw[ARC_MAX_FRAME_SIZE];
        int rawLen = arc_frame_build(raw,
                                     sizeof(raw),
                                     frame->src,
                                     frame->dst,
                                     frame->flags,
                                     frame->session,
                                     frame->seq,
                                     frame->family,
                                     frame->type,
                                     frame->payload,
                                     frame->payload_len);
        if (rawLen < 0)
            return;

        uint8_t encoded[ARC_MAX_ENCODED_SIZE];
        int encodedLen = arc_cobs_encode(raw, static_cast<size_t>(rawLen), encoded, sizeof(encoded));
        if (encodedLen < 0)
            return;

        stream->write(encoded, static_cast<size_t>(encodedLen));
    }

    void ArcCommandBridge::reliableSend(void *user, const arc_frame_t *frame)
    {
        ArcCommandBridge *bridge = static_cast<ArcCommandBridge *>(user);
        if (bridge)
            bridge->writeFrame(frame);
    }

    void ArcCommandBridge::reliableDeliver(void *user, const arc_frame_t *frame)
    {
        ArcCommandBridge *bridge = static_cast<ArcCommandBridge *>(user);
        if (bridge)
            bridge->handleDeliveredFrame(frame);
    }

    void ArcCommandBridge::reliableFail(void *user, const arc_frame_t *frame)
    {
        (void)user;
        (void)frame;
    }

} // namespace astra_rocket
