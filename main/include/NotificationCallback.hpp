#pragma once
#include <any>

template <typename T>
using OnDataCallback =
    std::function<void(const T &data, const std::any &userData)>;

template <typename T> struct CallbackInfo {
  OnDataCallback<T> callback;
  std::any userData;
};