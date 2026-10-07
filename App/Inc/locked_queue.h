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
        const bool overflow_flag = overflow_count;
        if (overflow_count > 0) overflow_count--;
        return overflow_flag;
    }

    void reset() {
        const bool irq = disable_irq();
        while (!std::queue<T>::empty()) std::queue<T>::pop();
        overflow_count = 0;
        restore_irq(irq);
    }

    bool push(const T& value) {
        const bool irq = disable_irq();
        bool ret = false;
        if (std::queue<T>::size() >= max_size) {
            overflow_count++;
        } else {
            std::queue<T>::push(value);
            ret = true;
        }
        restore_irq(irq);
        return ret;
    }

    void pop() {
        const bool irq = disable_irq();
        std::queue<T>::pop();
        restore_irq(irq);
    }

    bool front_pop_or_null(T *value) {
        const bool irq = disable_irq();
        if (!std::queue<T>::empty()) {
            *value = std::queue<T>::front();
            std::queue<T>::pop();
            restore_irq(irq);
            return true;
        }
        restore_irq(irq);
        return false;
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
    size_t overflow_count{false};
};
#endif

#endif //QCAN_QUEUE_WITH_IRQ_LOCK_H
