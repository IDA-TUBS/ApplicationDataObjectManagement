#ifndef SafeQueue_h
#define SafeQueue_h


#include <queue>
#include <mutex>
#include <condition_variable>

#include <adom/parameters.h>

/**
 * @brief A thread safe queue
 * 
 * @tparam T the data type to be stored in the queue
 */
template <class T>
class SafeQueue
{
    public:
    
    /**
     * @brief Construct a new (empty) Safe Queue object
     * 
     */
    SafeQueue(void)
    : safe_queue()
    , queue_lock()
    , queue_event()
    {}

    /**
     * @brief Destroy the Safe Queue object (default)
     * 
     */
    ~SafeQueue(void)
    {}

    /**
     * @brief Thread safe wrapper for std::queue push. A thread waiting on dequeue is notified after the enqueue operation
     * 
     * @param value the value to be enqueued
     */
    void enqueue(T value)
    {
        std::lock_guard<std::mutex> lock(queue_lock);
        safe_queue.push(value);
        queue_event.notify_one();
    }

    /**
     * @brief Get the "front"-element. If the queue is empty, wait till an element is available
     * 
     * @return T The dequeued value
     */
    T dequeue(void)
    {
        std::unique_lock<std::mutex> lock(queue_lock);
        while(safe_queue.empty())
        {
            // release lock as long as the wait and reaquire it afterwards.
            queue_event.wait(lock);
        }
        T val = safe_queue.front();
        safe_queue.pop();
        return val;
    }

    /**
     * @brief Get the "front"-element.
     * (!) Note: A non blocking call to an empty queue will lead to undefined behavior. Thus the empty() method needs to be called first.
     * (!) Note: A clean implementation would use exceptions to avoid a call to an empty queue.
     * 
     * @param blocking  
     * @return T The dequeued value
     */
    T dequeue(bool blocking = true)
    {
        T val;
        std::unique_lock<std::mutex> lock(queue_lock);
        if(blocking)
        {
            while(safe_queue.empty())
            {
                // release lock as long as the wait and reaquire it afterwards.
                queue_event.wait(lock);
            } 
        } else if (safe_queue.empty()) {
            throw EMPTY_QUEUE_ERROR;
        } 
        val = safe_queue.front();
        safe_queue.pop();
        return val;
        
    }

    T dequeue(bool blocking = true, std::chrono::milliseconds timeout = std::chrono::milliseconds::zero())
    {
        T val;
        std::unique_lock<std::mutex> lock(queue_lock);
        
        if (blocking)
        {
            if (timeout == std::chrono::milliseconds::zero())
            {
                // Old behavior, wait indefinitely
                while (safe_queue.empty())
                {
                    queue_event.wait(lock);
                }
            }
            else
            {
                // Wait for the specified timeout
                if (queue_event.wait_for(lock, timeout, [this] { return !safe_queue.empty(); }))
                {
                    // Successfully dequeued an item within timeout
                    val = safe_queue.front();
                    safe_queue.pop();
                    return val;
                }
                else
                {
                    // Timeout reached without dequeuing an item
                    throw std::runtime_error("Timeout reached while waiting for dequeuing");
                }
            }
        } 
        else 
        {
            if (safe_queue.empty())
            {
                // throw std::runtime_error("Queue was empty...");
            }
        }

        val = safe_queue.front();
        safe_queue.pop();
        return val;
    }

    /**
     * @brief Thread safe wrapper for std::queue empty()
     * 
     * @return true empty queue
     * @return false non empty queue
     */
    bool empty()
    {
        std::lock_guard<std::mutex> lock(queue_lock);
        return safe_queue.empty();
    }

    protected:
    
    /**
     * @brief The underlying queue which is accessed via the queue_lock (mutex)
     * 
     */
    std::queue<T> safe_queue;

    /**
     * @brief The mutex controlling the concurrent queue access
     * 
     */
    mutable std::mutex queue_lock;
    
    /**
     * @brief Condition variable used for an enqueue event. The blocking dequeue call waits for an enqueue event if the queue is empty.
     * 
     */
    std::condition_variable queue_event;
};



#endif




