#ifndef RTT_COMPOSITEFILTER_H
#define RTT_COMPOSITEFILTER_H

#include "Filter.h"

/**
 * The Composite Filter combines multiple filters and applies them in order.
 * @author Haico Dorenbos
 * @since 2019-12-2
 */
class CompositeFilter : Filter {
    private:
        std::vector<Filter*> filters = {}; // Stores all the filters

    public:
        /**
         * Constructor of the Composite Filter
         * @param filters All the filters which will be used on the data. The first filter in the array will be applied
         * first. The last filter in the array will be applied at last.
         */
        explicit CompositeFilter(std::vector<Filter*> &filters);

        google::protobuf::Message &filter(google::protobuf::Message &message) override;
};


#endif //RTT_COMPOSITEFILTER_H
