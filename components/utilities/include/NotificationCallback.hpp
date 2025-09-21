#pragma once
#include <functional>

template<typename KeyT, typename DataT, typename UserT>
using OnDataCallback
    = std::function<void(const KeyT &key, const DataT &data, const UserT &userData)>;

template<typename KeyT, typename DataT, typename UserT>
struct CallbackInfo
{
    uint32_t id;
    KeyT key;
    OnDataCallback<KeyT, DataT, UserT> callback;
    UserT userData;
};
