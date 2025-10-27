#ifndef RETRIEVE_DATA_H
#define RETRIEVE_DATA_H

namespace astra {
    class IRetrieveData {
    public:
        virtual bool init();
        virtual bool deleteFile(char *path);
        virtual void listFiles();
        virtual bool handleChoices();
        virtual bool readFile(char *path);
    };
}
#endif