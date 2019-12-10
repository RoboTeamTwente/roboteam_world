#include "filters/CompositeFilter.h"

void CompositeFilter::add(Filter *filter) {
    filters.push_back(filter);
};

google::protobuf::Message* CompositeFilter::filter(google::protobuf::Message* message) {
    google::protobuf::Message* result = message;
    for (auto &f : filters) {
        result->CopyFrom(*f->filter(result));
    }
    return result;
}