#include <string>
#include <sstream>
#include "ccParametersAPI.h"
 
class EDAPI
{
public:
    EDAPI()
    {
        ed_param_api = ed_get_cockpit_param_api();
    }
 
    //PARAMETER API
    void *getParamHandle(const char *name);
    void *getParamHandle(const std::string& name);
    void *getParamHandle(const std::stringstream& name);
    void setParam(void *handle, double value);
    void setParam(void *handle, const char *string);
    void setParam(void *handle, const std::string& string);
    void setParam(void *handle, const std::stringstream& ss);
    double getParam(void *handle);
    const char* getParam(void *handle, unsigned buffer_size);
    int compareParams(void *handle1, void *handle2);
 
    cockpit_param_api ed_param_api;
};
 
void *EDAPI::getParamHandle(const char *name)
{
    return ed_param_api.get_parameter_handle(name);
}
 
void *EDAPI::getParamHandle(const std::string& name)
{
    return ed_param_api.get_parameter_handle(name.c_str());
}
 
void *EDAPI::getParamHandle(const std::stringstream& name)
{
    return ed_param_api.get_parameter_handle(name.str().c_str());
}
 
void EDAPI::setParam(void *handle, double value)
{
    ed_param_api.update_parameter_with_number(handle, value);
}
 
void EDAPI::setParam(void *handle, const char *string)
{
    ed_param_api.update_parameter_with_string(handle, string);
}
 
 
void EDAPI::setParam(void *handle, const std::string& string)
{
    ed_param_api.update_parameter_with_string(handle, string.c_str());
}
 
 
void EDAPI::setParam(void *handle, const std::stringstream& ss)
{
    ed_param_api.update_parameter_with_string(handle, ss.str().c_str());
}
 
double EDAPI::getParam(void *handle)
{
    double res = 0;
    ed_param_api.parameter_value_to_number(handle, res, false);
    return res;
}
 
const char* EDAPI::getParam(void *handle, unsigned buffer_size)
{
    char buffer[256];
    ed_param_api.parameter_value_to_string(handle, buffer, 256);
    return &buffer[0];
}
 
int EDAPI::compareParams(void *handle1, void *handle2)
{
    return ed_param_api.compare_parameters(handle1, handle2);
}