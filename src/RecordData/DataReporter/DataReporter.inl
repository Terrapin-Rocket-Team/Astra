#ifndef DATA_REPORTER_INL
#define DATA_REPORTER_INL
namespace astra
{

    template <typename T>
    void DataReporter::insertColumn(int place, const char *fmt, T *variable, const char *label)
    {
        DataPoint *packedInfo = make_dp<T>(fmt, variable, label);

        if (!first)
        {
            first = last = packedInfo;
        }
        else if (place < 0 || place >= numColumns)
        { // append
            last->next = packedInfo;
            last = packedInfo;
        }
        else
        { // insert at index
            DataPoint *current = first;
            int idx = 0;
            while (current->next && idx < place - 1)
            {
                current = current->next;
                ++idx;
            }
            packedInfo->next = current->next;
            current->next = packedInfo;
            if (!packedInfo->next)
                last = packedInfo;
        }
        ++numColumns;
    }

    template <typename T>
    void DataReporter::addColumn(const char *fmt, T *variable, const char *label)
    {
        insertColumn<T>(-1, fmt, variable, label);
    }

} // namespace astra
#endif // DATA_REPORTER_INL
