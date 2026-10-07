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
}

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
        irqMutex lock_guard;
        while (!std::queue<T>::empty()) std::queue<T>::pop();
        overflow_count = 0;
    }

    bool push(const T& value) {
        irqMutex lock_guard;
        if (std::queue<T>::size() >= max_size) {
            overflow_count++;
        } else {
            std::queue<T>::push(value);
            return true;
        }
        return false;
    }

    void pop() {
        irqMutex lock_guard;
        std::queue<T>::pop();
    }

    bool front_pop_or_null(T *value) {
        irqMutex lock_guard;
        if (!std::queue<T>::empty()) {
            *value = std::queue<T>::front();
            std::queue<T>::pop();
            return true;
        }
        return false;
    }

    T back() const {
        irqMutex lock_guard;
        return std::queue<T>::back();
    }

    T front() const {
        irqMutex lock_guard;
        return std::queue<T>::front();
    }

    bool empty() const {
        irqMutex lock_guard;
        return std::queue<T>::empty();
    }

    bool full() const {
        irqMutex lock_guard;
        return std::queue<T>::size() >= max_size;
    }

    auto size() const {
        irqMutex lock_guard;
        return std::queue<T>::size();
    }

    void swap(queue_with_irq_lock& other) noexcept {
        irqMutex lock_guard;
        std::queue<T>::swap(other);
    }

    template <typename... Args>
    auto& emplace(Args&&... _args) {
        irqMutex lock_guard;
        return std::queue<T>::emplace(std::forward<Args>(_args)...);
    }

private:
    size_t max_size;
    size_t overflow_count{false};
};
#endif

#endif //QCAN_QUEUE_WITH_IRQ_LOCK_H
