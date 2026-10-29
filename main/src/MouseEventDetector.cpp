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
            || m->isButtonPressed(MouseButtonStateMachine::Button::Middle)) {
            return false;
        }
        if (m->isMoving()) {
            return false;
        }

        // Use ORIENTATION Z (yaw) for scroll wheel
        constexpr auto MAX_SCROLL_ANGLE{60.0f};
        constexpr auto SCROLL_SCALE{127.0f / MAX_SCROLL_ANGLE};
        // Clamp to the report range BEFORE narrowing to int8_t (see Mouse::saturateToReport).
        m->setAngle(Mouse::saturateToReport(SCROLL_SCALE * d.orientation.z));

        return m->isScrollWheelActive();
    }

    static bool handleButtonClick(const Qmi8658C::DataPoint &d, Mouse *m) {
        if (m->isScrollWheelActive()) {
            return false;
        }

        using MouseButtonStateMachine::Button::Left;
        using MouseButtonStateMachine::Button::Right;
        using MouseButtonStateMachine::Button::Middle;
        using MouseButtonStateMachine::Event::MouseDown;
        using MouseButtonStateMachine::Event::MouseUp;

        // Use angular speed (gyro) for button detection
        constexpr auto GYRO_PRESS_THRESHOLD{100.0f}; // Angular speed to trigger press
        constexpr auto LAST_RELEASE_THRESHOLD{pdMS_TO_TICKS(300)}; // Time to wait before releasing button
        static TickType_t lastMouseRelease = 0;

        const float gyroX = d.rawGyro.x; // Roll angular speed
        const float gyroY = d.rawGyro.y; // Pitch angular speed
        const bool isLeftRightClick = std::abs(gyroX) > GYRO_PRESS_THRESHOLD;
        const bool isVerticalClick = std::abs(gyroY) > GYRO_PRESS_THRESHOLD;
        const bool allButtonsIdle = m->isButtonIdle(Left)
                                    && m->isButtonIdle(Right)
                                    && m->isButtonIdle(Middle);

        // Listen for Press Events
        if (allButtonsIdle && (xTaskGetTickCount() - lastMouseRelease) > LAST_RELEASE_THRESHOLD) {
            if (isLeftRightClick) {
                if (gyroX < 0) {
                    m->getButton(Left).handleEvent(MouseDown);
                    return true;
                }
                if (gyroX > 0) {
                    m->getButton(Right).handleEvent(MouseDown);
                    return true;
                }
            }
            if (isVerticalClick && gyroY < 0) {
                m->getButton(Middle).handleEvent(MouseDown);
                return true;
            }
        }

        // Listen for Release Events
        if (isLeftRightClick) {
            if (gyroX > 0 && m->isButtonPressed(Left)) {
                m->getButton(Left).handleEvent(MouseUp);
                lastMouseRelease = xTaskGetTickCount();
                return true;
            }
            if (gyroX < 0 && m->isButtonPressed(Right)) {
                m->getButton(Right).handleEvent(MouseUp);
                lastMouseRelease = xTaskGetTickCount();
                return true;
            }
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
        if (m->isScrollWheelActive()) {
            return false;
        }

        // Use Gravity for mouse movement (rotation rates)
        constexpr auto GRAVITY_SCALE{127.0f / 9.8f}; // Sensitivity scaling
        const float rotationX = d.rawAccel.x;
        const float rotationY = d.rawAccel.y;

        // Clamp to the report range BEFORE narrowing to int8_t. Casting first wrapped around
        // for hard tilts and flipped the cursor direction (see Mouse::saturateToReport).
        m->updateMovement(Mouse::saturateToReport(GRAVITY_SCALE * rotationX),
                          Mouse::saturateToReport(GRAVITY_SCALE * rotationY));

        return m->isMoving();
    }

    // Template definition moved from the header; explicit instantiation provided below
    void MouseEventDetector::onSensorDataIsReady(const Qmi8658C::DataPoint &data, Mouse *mouse) {
        static ChainOfResponsibility<Qmi8658C::DataPoint, Mouse *> chain{};
        static bool initialized = false;

        if (!initialized) {
            // Order matters! Scroll wheel is checked first (while scrolling, buttons and movement
            // are suppressed), then buttons, then movement. The first handler returning true wins.
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
