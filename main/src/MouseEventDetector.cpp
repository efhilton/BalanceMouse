#include <ChainOfResponsibility.hpp>
#include <MouseEventDetector.hpp>
#include <cmath>
#include <memory>
#include <Qmi8658C.hpp>

namespace Hilton {

MouseEventDetector::MouseEventDetector() = default;

MouseEventDetector::~MouseEventDetector() = default;

template<typename DataT, typename UserDt>
static bool handleScrollWheel(const DataT &d, const std::shared_ptr<Mouse> &m)
{
    constexpr auto Z_START_THRESHOLD{10.0f};
    constexpr auto Z_STOP_THRESHOLD{8.0f};
    const float absOrientZ = std::abs(d.orientation.z);
    const bool isScrollWheelActive = m->isScrollWheelActive();

    const float clampedAngle = std::clamp(d.orientation.z, -90.0f, 90.0f);
    const float scaledWheel = (127.0f / 90.0f) * clampedAngle;
    const Mouse::ScrollWheelDt wheelValue = static_cast<Mouse::ScrollWheelDt>(std::round(std::clamp(scaledWheel, -127.0f, 127.0f)));

    // Implement a hysteresis
    if ((!isScrollWheelActive && absOrientZ > Z_START_THRESHOLD)                                       // trigger on
        || (isScrollWheelActive && absOrientZ > Z_STOP_THRESHOLD && absOrientZ < Z_START_THRESHOLD)) { // sustain in on position
        m->setScrollWheel(wheelValue);
        m->updateMovement(0, 0);
        // nothing more to be done.
        return true;
    }

    // No more scrolling.
    m->setScrollWheel(0);

    // Signal that more processing is needed.
    return false;
}

template<typename DataT, typename UserT>
static bool handleButtonClick(const DataT &d, const std::shared_ptr<Mouse> &m)
{
    using MouseButtonStateMachine::Button::Left;
    using MouseButtonStateMachine::Button::Middle;
    using MouseButtonStateMachine::Button::Right;
    using MouseButtonStateMachine::Event::MouseDown;
    using MouseButtonStateMachine::Event::MouseUp;

    constexpr auto X_PRESSED_THRESHOLD{10.0f};
    constexpr auto X_RELEASED_THRESHOLD(2.0f);
    constexpr auto X_DOT_THRESHOLD{55.0f};
    constexpr auto Y_PRESSED_THRESHOLD{10.0f};
    constexpr auto Y_RELEASED_THRESHOLD{2.0f};
    constexpr auto Y_DOT_THRESHOLD{55.0f};

    const bool isLeftButtonPressed = m->isButtonPressed(Left);
    const bool isMiddleButtonPressed = m->isButtonPressed(Middle);
    const bool isRightButtonPressed = m->isButtonPressed(Right);
    const bool isLeftButtonIdle = m->isButtonIdle(Left);
    const bool isMiddleButtonIdle = m->isButtonIdle(Middle);
    const bool isRightButtonIdle = m->isButtonIdle(Right);
    const bool isXButtonActionSpeed = std::abs(d.gyroscope.x) > X_DOT_THRESHOLD;
    const bool isYButtonActionSpeed = std::abs(d.gyroscope.y) > Y_DOT_THRESHOLD;
    const float deltaX = d.tilt.x - m->getMovement().deltaX;
    const float deltaY = d.tilt.y - m->getMovement().deltaY;

    if (isXButtonActionSpeed || isYButtonActionSpeed) {
        // the tilt toward the X or the Y imply that a button PRESS or RELEASE is coming.
        if (isXButtonActionSpeed && isLeftButtonPressed && deltaX > -X_RELEASED_THRESHOLD) {
            m->getButton(Left).handleEvent(MouseUp);
        } else if (isXButtonActionSpeed && isRightButtonPressed && deltaX < X_RELEASED_THRESHOLD) {
            m->getButton(Right).handleEvent(MouseUp);
        } else if (isYButtonActionSpeed && isMiddleButtonPressed && deltaY > -Y_RELEASED_THRESHOLD) {
            m->getButton(Middle).handleEvent(MouseUp);
        } else if (isLeftButtonIdle && isRightButtonIdle && isMiddleButtonIdle) {
            if (isXButtonActionSpeed && deltaX > X_PRESSED_THRESHOLD) {
                m->getButton(Right).handleEvent(MouseDown);
            } else if (isXButtonActionSpeed && deltaX < -X_PRESSED_THRESHOLD) {
                m->getButton(Left).handleEvent(MouseDown);
            } else if (isYButtonActionSpeed && deltaY < -Y_PRESSED_THRESHOLD) {
                m->getButton(Middle).handleEvent(MouseDown);
            }
        }
        m->updateMovement(0, 0);
        return true;
    }

    // Signal that more processing is needed.
    return false;
}
template<typename DataT, typename UserT>
static bool handleMouseMovement(const DataT &d, const std::shared_ptr<Mouse> &m)
{
    constexpr auto X_MOUSE_THRESHOLD{2.0f};
    constexpr auto X_MOUSE_SCALE{7.0f};
    constexpr auto Y_MOUSE_THRESHOLD{2.0f};
    constexpr auto Y_MOUSE_SCALE{-8.0f};

    const float deltaX = d.tilt.x - m->getMovement().deltaX;
    const float deltaY = d.tilt.y - m->getMovement().deltaY;

    if (std::abs(deltaX) > X_MOUSE_THRESHOLD || std::abs(deltaY) > Y_MOUSE_THRESHOLD) {
        // the tilt is slow, meaning no button presses are about to happen. Capture the mouse movement instead.
        const Mouse::MovementDt x = static_cast<Mouse::MovementDt>(std::round(std::clamp(X_MOUSE_SCALE * deltaX, -127.0f, 127.0f)));
        const Mouse::MovementDt y = static_cast<Mouse::MovementDt>(std::round(std::clamp(Y_MOUSE_SCALE * deltaY, -127.0f, 127.0f)));
        m->updateMovement(x, y);
        return true;
    }

    // Signal that more processing is needed.
    return false;
}

template<typename DataT, typename UserT>
static bool handleCatchAll(const DataT &d, const std::shared_ptr<Mouse> &m)
{
    m->updateMovement(0, 0);
    return false;
}

// Optional raw printer handler (can be disabled to save time)
template<typename DataT, typename UserT>
static bool handlePrintRaw(const DataT &d, const std::shared_ptr<Mouse> & /*ignored*/)
{
    printf(
        "Accel (%6.2f %6.2f %6.2f), Gyro (%6.2f, %6.2f, %6.2f)\n",
        d.accelerometer.x,
        d.accelerometer.y,
        d.accelerometer.z,
        d.gyroscope.x,
        d.gyroscope.y,
        d.gyroscope.z);
    return true;
}
// Template definition moved from the header; explicit instantiation provided below
template<typename KeyT, typename DataT, typename UserT>
void MouseEventDetector::onSensorDataIsReady(const KeyT &key, const DataT &data, const UserT &userData)
{
    // Static processing context and chain
    assert(key == Qmi8658C::QueueKey::DataReady);
    auto mouse = std::any_cast<std::shared_ptr<Mouse>>(userData);
    static ChainOfResponsibility<DataT, std::shared_ptr<Mouse>> chain{};
    static bool initialized = false;
    if (!initialized) {
        // Register handlers (enable what you need)
        chain.addHandler(handleScrollWheel<DataT, std::shared_ptr<Mouse>>);
        chain.addHandler(handleButtonClick<DataT, std::shared_ptr<Mouse>>);
        chain.addHandler(handleMouseMovement<DataT, std::shared_ptr<Mouse>>);
        chain.addHandler(handleCatchAll<DataT, std::shared_ptr<Mouse>>);
        chain.addHandler(handlePrintRaw<DataT, std::shared_ptr<Mouse>>);
        initialized = true;
    }

    chain.handle(data, mouse);
}

// Explicit instantiation for the currently used sensor types
template void MouseEventDetector::onSensorDataIsReady<Qmi8658C::QueueKey, Qmi8658C::DataPoint>(
    const Qmi8658C::QueueKey &, const Qmi8658C::DataPoint &, const std::any &);

} // namespace Hilton