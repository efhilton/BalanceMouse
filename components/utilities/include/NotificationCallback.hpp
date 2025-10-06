#pragma once
#include <functional>
#include <any>

// Callback signature for keyed notifications
template<typename KeyT, typename DataT, typename UserT = void *>
using OnDataCallback = std::function<void(const KeyT &key, const DataT &data, const UserT &userData)>;

// Information for a registered keyed callback. Store by value to avoid dangling references
// and to allow copying/assignment (important for containers and FreeRTOS usage).
template<typename KeyT, typename DataT, typename UserT = void *>
struct CallbackInfo {
    uint32_t id;
    KeyT key;
    OnDataCallback<KeyT, DataT, UserT> callback;
    UserT userData;
};

// Callback signature for simple (non-keyed) notifications
template<typename DataT, typename UserT = void *>
using OnSimpleCallback = std::function<void(const DataT &data, const UserT &userData)>;

// Information for a registered simple callback. Store by value for safe copying/moving.
template<typename DataT, typename UserT = void *>
struct SimpleCallbackInfo {
    uint32_t id;
    OnSimpleCallback<DataT, UserT> callback;
    UserT userData;
};
