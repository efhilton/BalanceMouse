#pragma once

#include <functional>
#include <memory>
#include <utility>

namespace Hilton {

template<typename DataDt, typename UserDt>
class ChainOfResponsibility
{
public:
    using CorHandler = std::function<bool(const DataDt &, const UserDt &)>;

    void addHandler(const CorHandler &handler)
    {
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

    void deleteHandler(const CorHandler &handler)
    {
        auto current = head;
        auto prev = std::shared_ptr<Node>();

        while (current) {
            if (current->handler == handler) {
                if (current == head) {
                    head = current->next;
                    current = head;
                } else {
                    prev->next = current->next;
                    current = current->next;
                }
            } else {
                prev = current;
                current = current->next;
            }
        }
    }

    bool handle(const DataDt &event, const UserDt &context) const
    {
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
    struct Node
    {
        explicit Node(CorHandler handler)
            : handler(std::move(handler))
        {}
        CorHandler handler;
        std::shared_ptr<Node> next;
    };

    std::shared_ptr<Node> head; // No more unique ownership; use shared across functions
};

} 