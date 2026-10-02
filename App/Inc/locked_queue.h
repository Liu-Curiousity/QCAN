/**
 * @brief 		queue_with_irq_lock.h库文件
 * @detail
 * @author 	    Haoqi Liu
 * @date        2026/10/2
 * @version 	V1.0.0
 * @note 		
 * @warning	    
 * @par 		历史版本
                V1.0.0创建于2026/10/2
 * */

#ifndef QCAN_QUEUE_WITH_IRQ_LOCK_H
#define QCAN_QUEUE_WITH_IRQ_LOCK_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
};

#include <queue>
#include "util.h"

template <typename T>
class queue_with_irq_lock : public std::queue<T> {
public:
    explicit queue_with_irq_lock(const size_t max_size) : max_size(max_size) {}

    bool is_overflow() {
        const bool overflow = overflow_flag;
        if (overflow_flag > 0) overflow_flag--;
        return overflow;
    }

    bool push(const T& value) {
        const bool irq = disable_irq();
        overflow_flag += std::queue<T>::size() >= max_size;
        if (!overflow_flag) std::queue<T>::push(value);
        restore_irq(irq);
        return !overflow_flag;
    }

    void pop() {
        const bool irq = disable_irq();
        std::queue<T>::pop();
        restore_irq(irq);
    }

    T back() const {
        const bool irq = disable_irq();
        const T result = std::queue<T>::back();
        restore_irq(irq);
        return result;
    }

    T front() const {
        const bool irq = disable_irq();
        const T result = std::queue<T>::front();
        restore_irq(irq);
        return result;
    }

    bool empty() const {
        const bool irq = disable_irq();
        const bool result = std::queue<T>::empty();
        restore_irq(irq);
        return result;
    }

    bool full() const {
        const bool irq = disable_irq();
        const bool result = std::queue<T>::size() >= max_size;
        restore_irq(irq);
        return result;
    }

    auto size() const {
        const bool irq = disable_irq();
        const auto result = std::queue<T>::size();
        restore_irq(irq);
        return result;
    }

    void swap(queue_with_irq_lock& other) noexcept {
        const bool irq = disable_irq();
        std::queue<T>::swap(other);
        restore_irq(irq);
    }

    template <typename... Args>
    auto& emplace(Args&&... _args) {
        const bool irq = disable_irq();
        auto& result = std::queue<T>::emplace(std::forward<Args>(_args)...);
        restore_irq(irq);
        return result;
    }

private:
    size_t max_size;
    size_t overflow_flag{false};
};
#endif

#endif //QCAN_QUEUE_WITH_IRQ_LOCK_H
