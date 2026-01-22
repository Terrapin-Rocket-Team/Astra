#include "SerialMessageRouter.h"
#include <cstring>

namespace astra
{

SerialMessageRouter::SerialMessageRouter(size_t maxInterfaces, size_t maxPrefixes, size_t bufferSize)
    : interfaces(nullptr)
    , listeners(nullptr)
    , defaultHandler(nullptr)
    , maxInterfaces(maxInterfaces)
    , maxPrefixes(maxPrefixes)
    , interfaceCount(0)
    , listenerCount(0)
    , defaultBufferSize(bufferSize)
    , delimiter('\n')
{
    // Allocate interface array
    interfaces = new SerialInterface[maxInterfaces];

    // Allocate listener array
    listeners = new PrefixListener[maxPrefixes];
}

SerialMessageRouter::~SerialMessageRouter()
{
    // Free all interface buffers
    if (interfaces) {
        for (size_t i = 0; i < interfaceCount; i++) {
            if (interfaces[i].buffer) {
                delete[] interfaces[i].buffer;
            }
        }
        delete[] interfaces;
    }

    // Free listener array
    if (listeners) {
        delete[] listeners;
    }
}

SerialMessageRouter& SerialMessageRouter::withInterface(Stream* stream)
{
    if (!stream || interfaceCount >= maxInterfaces) {
        return *this;
    }

    // Check if already registered
    if (findInterface(stream) >= 0) {
        return *this;
    }

    // Add new interface
    SerialInterface& iface = interfaces[interfaceCount];
    iface.stream = stream;
    iface.bufferSize = defaultBufferSize;
    iface.buffer = new char[defaultBufferSize];
    iface.bufferPos = 0;
    iface.buffer[0] = '\0';

    interfaceCount++;
    return *this;
}

SerialMessageRouter& SerialMessageRouter::withListener(const char* prefix, MessageCallback callback)
{
    if (!prefix || !callback || listenerCount >= maxPrefixes) {
        return *this;
    }

    // Add new listener
    listeners[listenerCount] = PrefixListener(prefix, callback);
    listenerCount++;
    return *this;
}

SerialMessageRouter& SerialMessageRouter::withDelimiter(char delim)
{
    delimiter = delim;
    return *this;
}

SerialMessageRouter& SerialMessageRouter::withDefaultHandler(MessageCallback callback)
{
    defaultHandler = callback;
    return *this;
}

void SerialMessageRouter::update()
{
    // Poll each registered interface
    for (size_t i = 0; i < interfaceCount; i++) {
        SerialInterface& iface = interfaces[i];

        if (!iface.stream) {
            continue;
        }

        // Read all available bytes
        while (iface.stream->available()) {
            int c = iface.stream->read();

            if (c < 0) {
                break; // No more data
            }

            // Check for delimiter
            if (c == delimiter) {
                // Null-terminate the buffer
                iface.buffer[iface.bufferPos] = '\0';

                // Process the complete line
                if (iface.bufferPos > 0) {
                    processLine(iface.buffer, iface.stream);
                }

                // Reset buffer
                iface.bufferPos = 0;
            }
            else {
                // Add character to buffer if there's room
                if (iface.bufferPos < iface.bufferSize - 1) {
                    iface.buffer[iface.bufferPos++] = (char)c;
                }
                else {
                    // Buffer overflow - reset and skip this line
                    iface.bufferPos = 0;
                    // Could add error logging here if needed
                }
            }
        }
    }
}

void SerialMessageRouter::processLine(const char* line, Stream* source)
{
    if (!line || !source) {
        return;
    }

    // Try to match against all registered prefixes
    bool matched = false;
    for (size_t i = 0; i < listenerCount; i++) {
        const PrefixListener& listener = listeners[i];

        if (matchesPrefix(line, listener.prefix)) {
            // Found a match - extract message after prefix
            const char* message = line + strlen(listener.prefix);

            // Call the callback
            listener.callback(message, listener.prefix, source);

            matched = true;
            break; // Only match first prefix (in registration order)
        }
    }

    // If no prefix matched and there's a default handler, call it
    if (!matched && defaultHandler) {
        defaultHandler(line, "", source);
    }
}

bool SerialMessageRouter::matchesPrefix(const char* line, const char* prefix) const
{
    if (!line || !prefix) {
        return false;
    }

    size_t prefixLen = strlen(prefix);
    if (prefixLen == 0) {
        return false;
    }

    return strncmp(line, prefix, prefixLen) == 0;
}

int SerialMessageRouter::findInterface(Stream* stream) const
{
    for (size_t i = 0; i < interfaceCount; i++) {
        if (interfaces[i].stream == stream) {
            return (int)i;
        }
    }
    return -1;
}

} // namespace astra
