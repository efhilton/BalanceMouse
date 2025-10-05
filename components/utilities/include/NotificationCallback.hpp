#pragma once
#include <functional>
#include <any>

template<typename KeyT, typename DataT, typename UserT = std::any>
using OnDataCallback = std::function<void(const KeyT &key, const DataT &data, const UserT &userData)>;

template<typename KeyT, typename DataT, typename UserT = std::any>
struct CallbackInfo {
    const uint32_t id;
    const KeyT key;
    const OnDataCallback<KeyT, DataT, UserT> &callback;
    const UserT &userData;
};

template<typename DataT, typename UserT = std::any>
using OnSimpleCallback = std::function<void(const DataT &data, const UserT &userData)>;

template<typename DataT, typename UserT = std::any>
struct SimpleCallbackInfo {
    const uint32_t id;
    const OnSimpleCallback<DataT, UserT> &callback;
    const UserT &userData;
};
