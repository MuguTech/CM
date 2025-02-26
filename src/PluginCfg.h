#ifndef __BASE_CFG_H__
#define __BASE_CFG_H__

#include <string>


struct _socket
{
    std::string name;
    std::string ip;
    std::string port;
    std::string type;
};


class PluginCfg
{
private:
    std::string name;
    std::string type;
    std::string id;
    std::string lib;

public:
    PluginCfg();
    ~PluginCfg();

    virtual std::string get_name(){return name;}
    virtual std::string get_type(){return type;}
    virtual std::string get_id(){return id;}
    virtual std::string get_lib(){return lib;}

    virtual void set_name(const std::string &name){this->name = name;}
    virtual void set_type(const std::string type){this->type = type;}
    virtual void set_id(const std::string id){this->id = id;}
    virtual void set_lib(const std::string lib){this->lib = lib;}

};





#endif  // __BASE_CFG_H__

