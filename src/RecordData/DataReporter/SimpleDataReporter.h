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
        using UpdateFuncCB = T(*)();
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
            T defaultValue = nullptr) :
            DataReporter(name), _beginFunc(beginFunc), _updateFunc(updateFunc), loggedVariable(defaultValue) 
        {
            addColumn(fmt, &loggedVariable, label);
        }
        virtual ~SimpleDataReporter() = default;

       

    protected:

        bool init() override
        {
            if (_beginFunc)
            {
                return _beginFunc();
            }
            LOGE("Data Reporter %s was not given an init function", getName());
            return false;
        }


        bool read() override {
            if (_updateFunc)
            {
                loggedVariable = _updateFunc();
                return true;
            }
            return false;
        };

    private:
        BeginFuncCB _beginFunc;
        UpdateFuncCB _updateFunc;
        T loggedVariable;
    };
    
}
#endif
