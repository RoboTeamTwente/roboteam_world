#include "CompositeFilter.h"

CompositeFilter::CompositeFilter(std::vector<Filter*> &filters) {
    this->filters = filters;
}

google::protobuf::Message &CompositeFilter::filter(google::protobuf::Message &message) {
    google::protobuf::Message &result = message;
    for (auto &f : filters) {
        result.CopyFrom(f->filter(result));
    }
    return result;
}