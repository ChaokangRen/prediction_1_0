#pragma once

#include <string>

namespace jarvis {
namespace prediction_lib {

class PredictorInterface {
public:
    virtual ~PredictorInterface(){};

    static PredictorInterface *CreateInstance();

    virtual bool Init() = 0;

    virtual bool Execute() = 0;
};

}  // namespace prediction_lib
}  // namespace jarvis