#ifndef SIMPLE_DATA_REPORTER_H
#define SIMPLE_DATA_REPORTER_H
#include "RecordData/Logging/EventLogger.h"
#include "DataReporter.h"

namespace astra
{
    /**
     * @brief A simplified DataReporter that uses function callbacks for initialization and updates.
     *
     * This class makes it easy to add simple single-datapoint logging without creating a full class.
     * Just provide begin() and update() functions, and the reporter handles the rest.
     */
    template <typename T>
    class SimpleDataReporter : public DataReporter
    {
    public:
        using BeginFuncCB = bool (*)();
        using UpdateFuncCB = T (*)();
        using BeginFuncWithContextCB = bool (*)(void *);
        using UpdateFuncWithContextCB = T (*)(void *);
        /**
         * @brief Construct a new Simple Data Reporter with function callbacks
         *
         * @param name Name for this reporter (appears in CSV headers)
         * @param beginFunc Function to call during initialization (can be nullptr)
         * @param updateFunc Function to call during updates (can be nullptr)
         */
        SimpleDataReporter(
            const char* name = "Simple Data Reporter",
            const char* fmt = "%f", 
            const char* label = "Default Label",
            BeginFuncCB beginFunc = nullptr,
            UpdateFuncCB updateFunc = nullptr,
            T defaultValue = T{}) :
            DataReporter(name), _beginFunc(beginFunc), _updateFunc(updateFunc), loggedVariable(defaultValue) 
        {
            addColumn(fmt, &loggedVariable, label);
        }

        SimpleDataReporter(
            const char *name,
            const char *fmt,
            const char *label,
            BeginFuncWithContextCB beginFunc,
            UpdateFuncWithContextCB updateFunc,
            void *context,
            T defaultValue = T{}) :
            DataReporter(name), _beginFuncWithContext(beginFunc), _updateFuncWithContext(updateFunc), _context(context), loggedVariable(defaultValue)
        {
            addColumn(fmt, &loggedVariable, label);
        }
        virtual ~SimpleDataReporter() = default;

        int begin() override
        {
            if (_beginFuncWithContext)
            {
                initialized = _beginFuncWithContext(_context);
                return initialized ? 0 : -1;
            }
            if (_beginFunc)
            {
                initialized = _beginFunc();
                return initialized ? 0 : -1;
            }
            LOGE("Data Reporter %s was not given an init function", getName());
            return -1;
        }

        int update() override
        {
            if (_updateFuncWithContext)
            {
                loggedVariable = _updateFuncWithContext(_context);
                return 0;
            }
            if (_updateFunc)
            {
                loggedVariable = _updateFunc();
                return 0;
            }
            return -1;
        }

    private:
        BeginFuncCB _beginFunc = nullptr;
        UpdateFuncCB _updateFunc = nullptr;
        BeginFuncWithContextCB _beginFuncWithContext = nullptr;
        UpdateFuncWithContextCB _updateFuncWithContext = nullptr;
        void *_context = nullptr;
        T loggedVariable;
    };
    
}
#endif
