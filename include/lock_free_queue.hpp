/* This file is part of LRT-MPICPF - (https://github.com/gollertim/lrt_mpicpf)
 *
 * LRT-MPICPF -- A software framework for model predictive interaction control based on a path-following formulation (MPIC-PF) for robotic manipulation tasks
 *
 * Copyright 2025 Tim Goller, Tobias Gold, Andreas Voelz, Knut Graichen.
 * All rights reserved.
 *
 * LRT-MPICPF is distributed under the BSD-3-Clause license, see LICENSE.txt
 *
 */

#ifndef LOCK_FREE_QUEUE_H
#define LOCK_FREE_QUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>

namespace lrt_mpic
{

    /**
     * This class can be used for buffering and logging data
    */
    template <typename T>
    class LockFreeQueue
    {
    public:
        /**
         * @brief Pushes an item into the queue.
         * 
         * This method adds an item to the queue in a thread-safe manner.
         * It uses a mutex to ensure exclusive access to the queue and notifies
         * one waiting thread (if any) that a new item is available.
         * 
         * @param item The item to be pushed into the queue.
         */
        void push(const T &item)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(item);
            cond_var_.notify_one();
        }

        /**
         * @brief Attempts to pop an item from the queue.
         * 
         * This method tries to remove an item from the queue in a thread-safe manner.
         * If the queue is empty, it returns false without modifying the item.
         * 
         * @param item Reference to a variable where the popped item will be stored.
         * @return true If an item was successfully popped from the queue.
         * @return false If the queue was empty.
         */
        bool pop(T &item)
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (queue_.empty())
            {
                return false;
            }
            item = std::move(queue_.front());
            queue_.pop();
            return true;
        }

        /**
         * @brief Waits for an item to be available and pops it from the queue.
         * 
         * This method blocks the calling thread until an item is available in the queue.
         * Once an item is available, it removes the item in a thread-safe manner.
         * 
         * @param item Reference to a variable where the popped item will be stored.
         */
        void wait_and_pop(T &item)
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cond_var_.wait(lock, [this]
                           { return !queue_.empty(); });
            item = std::move(queue_.front());
            queue_.pop();
        }

    private:
        std::queue<T> queue_;
        std::mutex mutex_;
        std::condition_variable cond_var_;
    };

}

#endif // LOCK_FREE_QUEUE_H