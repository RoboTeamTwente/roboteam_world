#ifndef RTT_COMPOSITEFILTER_H
#define RTT_COMPOSITEFILTER_H

#include "Filter.h"

/**
 * The Composite Filter combines multiple filters and applies them in order.
 * @author Haico Dorenbos
 * @since 2019-12-2
 */
class CompositeFilter : public Filter {
    private:
        std::vector<Filter*> filters = {}; // Stores all the filters

    public:
        CompositeFilter() = default;

        /**
         * Adds a filter to the Composite Filter. (The first filter added will be applied first on the data)
         * @param filter The added filter.
         */
        void add(Filter *filter);

        google::protobuf::Message* filter(google::protobuf::Message* message) override;
};


#endif //RTT_COMPOSITEFILTER_H
