#include <ChainOfResponsibility.hpp>
#include <MouseEventDetector.hpp>
#include <cmath>
#include <memory>
#include <Qmi8658C.hpp>
#include <lwip/prot/ip6.h>

namespace Hilton {
    MouseEventDetector::MouseEventDetector() = default;

    MouseEventDetector::~MouseEventDetector() = default;

    static bool handleScrollWheel(const Qmi8658C::DataPoint &d, Mouse *m) {
        if (m->isButtonPressed(MouseButtonStateMachine::Button::Left)
            || m->isButtonPressed(MouseButtonStateMachine::Button::Right)
            || m->isButtonPressed(MouseButtonStateMachine::Button::Middle)
            || m->isButtonPressed(MouseButtonStateMachine::Button::Wheel)) {
            return false;
        }

        // Use ORIENTATION Z (yaw) for scroll wheel
        constexpr auto MAX_SCROLL_ANGLE{60.0f};
        constexpr auto SCROLL_SCALE{127.0f / MAX_SCROLL_ANGLE};
        m->setAngle(static_cast<Mouse::ScrollWheelDt>(SCROLL_SCALE * d.orientation.z));

        if (m->isScrollWheelActive()) {
            m->updateMovement(0, 0); // Stop cursor movement during scroll
            return true; // signal that we are done proessing.
        }

        m->setAngle(0);
        return false;
    }

    static bool handleButtonClick(const Qmi8658C::DataPoint &d, Mouse *m) {
        using MouseButtonStateMachine::Button::Left;
        using MouseButtonStateMachine::Button::Right;
        using MouseButtonStateMachine::Button::Middle;
        using MouseButtonStateMachine::Event::MouseDown;
        using MouseButtonStateMachine::Event::MouseUp;

        // Use angular speed (gyro) for button detection
        constexpr auto GYRO_PRESS_THRESHOLD{100.0f}; // Angular speed to trigger press
        constexpr auto LAST_RELEASE_THRESHOLD{pdMS_TO_TICKS(200)}; // Time to wait before releasing button
        static TickType_t lastMouseRelease = 0;

        const float gyroX = d.rawGyro.x; // Roll angular speed
        const float gyroY = d.rawGyro.y; // Pitch angular speed
        const bool isLeftRightClick = std::abs(gyroX) > GYRO_PRESS_THRESHOLD;
        const bool isVerticalClick = std::abs(gyroY) > GYRO_PRESS_THRESHOLD;

        // LEFT BUTTON (Negative X angular speed)
        if (isLeftRightClick && gyroX < 0
            && m->isButtonIdle(Left)
            && m->isButtonIdle(Right)
            && (xTaskGetTickCount() - lastMouseRelease) > LAST_RELEASE_THRESHOLD) {
            m->getButton(Left).handleEvent(MouseDown);
            return true;
        }
        if (isLeftRightClick && gyroX > 0
            && m->isButtonPressed(Left)) {
            m->getButton(Left).handleEvent(MouseUp);
            lastMouseRelease = xTaskGetTickCount();
            return true;
        }

        // RIGHT BUTTON (Positive X angular speed)
        if (isLeftRightClick && gyroX > 0
            && m->isButtonIdle(Right)
            && m->isButtonIdle(Left)
            && (xTaskGetTickCount() - lastMouseRelease) > LAST_RELEASE_THRESHOLD) {
            m->getButton(Right).handleEvent(MouseDown);
            return true;
        }
        if (isLeftRightClick && gyroX < 0
            && m->isButtonPressed(Right)) {
            m->getButton(Right).handleEvent(MouseUp);
            lastMouseRelease = xTaskGetTickCount();
            return true;
        }

        // MIDDLE BUTTON (Negative Y angular speed)
        if (isVerticalClick && gyroY < 0
            && m->isButtonIdle(Middle)
            && (xTaskGetTickCount() - lastMouseRelease) > LAST_RELEASE_THRESHOLD) {
            m->getButton(Middle).handleEvent(MouseDown);
            return true;
        }
        if (isVerticalClick && gyroY > 0
            && m->isButtonPressed(Middle)) {
            m->getButton(Middle).handleEvent(MouseUp);
            lastMouseRelease = xTaskGetTickCount();
            return true;
        }

        return false;
    }

    static bool handleMouseMovement(const Qmi8658C::DataPoint &d, Mouse *m) {
        // Use Gravity for mouse movement (rotation rates)
        constexpr auto GRAVITY_SCALE{127.0f / 9.8f}; // Sensitivity scaling

        const float rotationX = d.rawAccel.x; // Roll rate
        const float rotationY = d.rawAccel.y; // Pitch rate

        m->updateMovement(static_cast<Mouse::MovementDt>(GRAVITY_SCALE * rotationX),
                          static_cast<Mouse::MovementDt>(GRAVITY_SCALE * rotationY));

        return m->isMoving();
    }

    // Template definition moved from the header; explicit instantiation provided below
    void MouseEventDetector::onSensorDataIsReady(const Qmi8658C::DataPoint &data, Mouse *mouse) {
        static ChainOfResponsibility<Qmi8658C::DataPoint, Mouse *> chain{};
        static bool initialized = false;

        if (!initialized) {
            // Order matters! Button detection has highest priority
            chain.addHandler(handleScrollWheel);
            chain.addHandler(handleButtonClick);
            chain.addHandler(handleMouseMovement);
            initialized = true;
        }

        if (chain.handle(data, mouse)) {
            simpleNotificationService.notify(mouse);
        }
    }
}
