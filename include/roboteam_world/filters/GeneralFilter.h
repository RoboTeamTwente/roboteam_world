#ifndef RTT_GENERALFILTER_H
#define RTT_GENERALFILTER_H

#include "filters/CompositeFilter.h"

/**
 * The General filter is the filter that we apply on all proto messages.
 * @author Haico Dorenbos
 * @since 2019-12-2
 */
class GeneralFilter : public CompositeFilter {
    public:
        GeneralFilter();
};

#endif //RTT_GENERALFILTER_H
