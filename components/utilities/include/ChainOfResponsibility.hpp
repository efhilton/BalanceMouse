#pragma once

#include <functional>
#include <memory>

namespace Hilton {

template<typename Data, typename Context>
class ChainOfResponsibility
{
public:
    using Callback = std::function<bool(const Data&, Context &)>;

    void addHandler(Callback handler) {
        auto newNode = std::make_shared<Node>(std::move(handler));
        if (!head) {
            head = newNode;
        } else {
            auto current = head;
            while (current->next) {
                current = current->next;
            }
            current->next = newNode;
        }
    }

    bool handle(const Data& event, Context &context) const {
        auto current = head;
        while (current) {
            if (current->handler(event, context)) {
                return true;
            }
            current = current->next;
        }
        return false;
    }

private:
    struct Node {
        explicit Node(Callback handler) : handler(std::move(handler)) {}
        Callback handler;
        std::shared_ptr<Node> next;
    };

    std::shared_ptr<Node> head; // No more unique ownership; use shared across functions
};

} // namespace Hilton