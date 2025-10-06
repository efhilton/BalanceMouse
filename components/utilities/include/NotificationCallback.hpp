#pragma once
#include <functional>

/**
 * @brief Callback signature for notifications with key-based filtering
 * @tparam KeyT Type of the key used for filtering notifications
 * @tparam DataT Type of the data payload in the notification
 * @tparam UserT Type of the user data passed to callback (defaults to void*)
 * @param key The key associated with the notification
 * @param data The data payload of the notification
 * @param userData User-defined data passed through to callback
 */
template<typename KeyT, typename DataT, typename UserT = void *>
using OnDataCallback = std::function<void(const KeyT &key, const DataT &data, const UserT &userData)>;

/**
 * @brief Callback signature for simple notifications without key filtering
 * @tparam DataT Type of the data payload in the notification
 * @tparam UserT Type of the user data passed to callback (defaults to void*)
 * @param data The data payload of the notification
 * @param userData User-defined data passed through to callback
 */
template<typename DataT, typename UserT = void *>
using OnSimpleCallback = std::function<void(const DataT &data, const UserT &userData)>;
