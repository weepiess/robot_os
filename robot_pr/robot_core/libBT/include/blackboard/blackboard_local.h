#ifndef BLACKBOARD_LOCAL_H
#define BLACKBOARD_LOCAL_H

#include "basic_blackboard.h"

namespace bt
{
class BlackboardLocal : public BlackboardImpl
{
  public:
    BlackboardLocal()
    {
    }

    virtual const SafeAny::Any* get(const std::string& key) const override
    {
        auto it = storage_.find(key);
        if (it == storage_.end())
        {
            return nullptr;
        }
        return &(it->second);
    }

    virtual void set(const std::string& key, const SafeAny::Any& value) override
    {
        storage_[key] = value;
    }

    virtual bool contains(const std::string& key) const override
    {
        return storage_.find(key) != storage_.end();
    }

  private:
    std::unordered_map<std::string, SafeAny::Any> storage_;
};
}

#endif   // BLACKBOARD_LOCAL_H
