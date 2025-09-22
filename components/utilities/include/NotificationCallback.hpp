#pragma once
#include <functional>

template<typename KeyT, typename DataT, typename UserT = void*>
using OnDataCallback
    = std::function<void(const KeyT &key, const DataT &data, const UserT &userData)>;

template<typename KeyT, typename DataT, typename UserT = void*>
struct CallbackInfo
{
    uint32_t id;
    KeyT key;
    OnDataCallback<KeyT, DataT, UserT> callback;
    UserT userData;
};

template<typename DataT, typename UserT = void*>
using OnSimpleCallback = std::function<void(const DataT &data, const UserT &userData)>;

template<typename DataT, typename UserT = void*>
struct SimpleCallbackInfo
{
    uint32_t id;
    OnSimpleCallback<DataT, UserT> callback;
    UserT userData;
};