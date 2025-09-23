#include "../../src/RecordData/DataReporter/DataReporter.h"

namespace astra
{
    class testDR : public DataReporter
    {
    public:
        DataPoint *first = nullptr, *last = nullptr;
        int packedDataSize = 0;
        uint8_t numColumns = 0;
        uint8_t *packedData = nullptr;
        int var1 = 0;
        double var2 = 0;
        testDR(){
            addColumn("%d", &var1, "int");
            addColumn("%0.3f", &var2, "DtoF");
            addColumn("%0.7f", &var2, "Double");
            removeColumn("DtoF");
            insertColumn(1, "%0.3f", &var2, "DtoF2");
        }
    };
}