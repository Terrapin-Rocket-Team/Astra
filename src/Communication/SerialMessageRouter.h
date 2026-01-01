#ifndef SERIAL_MESSAGE_ROUTER_H
#define SERIAL_MESSAGE_ROUTER_H

#include <Arduino.h>

namespace astra
{

/**
 * Callback function signature for handling serial messages.
 * @param message The message content after the prefix has been stripped
 * @param prefix The prefix that matched this message
 * @param source Pointer to the Stream that received the message
 */
typedef void (*MessageCallback)(const char* message, const char* prefix, Stream* source);

/**
 * Represents a single prefix listener configuration
 */
struct PrefixListener {
    const char* prefix;
    MessageCallback callback;

    PrefixListener() : prefix(nullptr), callback(nullptr) {}
    PrefixListener(const char* p, MessageCallback cb) : prefix(p), callback(cb) {}
};

/**
 * Represents a serial interface being monitored
 */
struct SerialInterface {
    Stream* stream;
    char* buffer;
    size_t bufferPos;
    size_t bufferSize;

    SerialInterface() : stream(nullptr), buffer(nullptr), bufferPos(0), bufferSize(0) {}
};

/**
 * SerialMessageRouter - Centralized serial message handler
 *
 * Monitors multiple serial interfaces (UART, USB, etc.) and dispatches
 * messages to registered callbacks based on prefix matching.
 *
 * Features:
 * - Multiple serial interfaces supported
 * - Any prefix can appear on any interface
 * - Builder pattern for easy configuration
 * - Non-blocking operation
 * - Automatic buffer management
 */
class SerialMessageRouter {
public:
    /**
     * Constructor
     * @param maxInterfaces Maximum number of serial interfaces to monitor (default: 4)
     * @param maxPrefixes Maximum number of prefixes to register (default: 8)
     * @param bufferSize Size of line buffer per interface (default: 256)
     */
    SerialMessageRouter(size_t maxInterfaces = 4, size_t maxPrefixes = 8, size_t bufferSize = 256);

    /**
     * Destructor - cleans up allocated buffers
     */
    ~SerialMessageRouter();

    /**
     * Register a serial interface to monitor
     * @param stream Pointer to Stream object (Serial, Serial1, etc.)
     * @return Reference to this for method chaining
     */
    SerialMessageRouter& withInterface(Stream* stream);

    /**
     * Register a prefix listener (works on all interfaces)
     * @param prefix The prefix to match (e.g., "HITL/", "CMD/", "RAD/")
     * @param callback Function to call when prefix is matched
     * @return Reference to this for method chaining
     */
    SerialMessageRouter& withListener(const char* prefix, MessageCallback callback);

    /**
     * Set the line delimiter character (default: '\n')
     * @param delim The delimiter character
     * @return Reference to this for method chaining
     */
    SerialMessageRouter& withDelimiter(char delim);

    /**
     * Set a default handler for messages that don't match any prefix
     * @param callback Function to call for unmatched messages
     * @return Reference to this for method chaining
     */
    SerialMessageRouter& withDefaultHandler(MessageCallback callback);

    /**
     * Main update function - call this in your loop()
     * Polls all registered serial interfaces and dispatches callbacks
     */
    void update();

    /**
     * Get the number of registered interfaces
     */
    size_t getInterfaceCount() const { return interfaceCount; }

    /**
     * Get the number of registered prefix listeners
     */
    size_t getListenerCount() const { return listenerCount; }

private:
    SerialInterface* interfaces;
    PrefixListener* listeners;
    MessageCallback defaultHandler;

    size_t maxInterfaces;
    size_t maxPrefixes;
    size_t interfaceCount;
    size_t listenerCount;
    size_t defaultBufferSize;
    char delimiter;

    /**
     * Process a complete line from a serial interface
     * @param line The complete line (null-terminated)
     * @param source The Stream that received the line
     */
    void processLine(const char* line, Stream* source);

    /**
     * Check if a line matches a prefix
     * @param line The line to check
     * @param prefix The prefix to match
     * @return true if the line starts with the prefix
     */
    bool matchesPrefix(const char* line, const char* prefix) const;

    /**
     * Find the interface index for a given stream
     * @param stream The stream to find
     * @return Index of the interface, or -1 if not found
     */
    int findInterface(Stream* stream) const;
};

} // namespace astra

#endif // SERIAL_MESSAGE_ROUTER_H
