#include <Arduino.h>

enum TimeUnit {
    SECONDS = 0,
    MILLISECONDS = 1
};

class MillisTimer {

    private:
        uint32_t mStartTime;
        uint32_t mEndTime;
        uint32_t mLastFrameTime;
        uint32_t mLastLastFrameTime;
        bool mRunning = true;

        uint32_t GetTimeElapsed(uint32_t t1, uint32_t t2, TimeUnit unit = TimeUnit::SECONDS) {
            return unit == TimeUnit::SECONDS ? (t1 - t2) / 1000 : (t1 - t2);
        }

    public: 
        MillisTimer() {
            Start();
            Frame();
        }

        void Start() {
            mStartTime = millis();
            mLastFrameTime = mStartTime;
            mLastLastFrameTime = mStartTime;
            mRunning = true;
        }

        void Stop() {
            mEndTime = millis();
            mRunning = false;
        }
        
        void Frame() {
            if (mRunning) {
                mLastLastFrameTime = mLastFrameTime;
                mLastFrameTime = millis(); 
            }
        }

        double GetLastFrameElapsed(TimeUnit unit = TimeUnit::SECONDS) {
            return GetTimeElapsed(mLastFrameTime, mLastLastFrameTime);
        }

        double GetCurrentFrameElapsed(TimeUnit unit = TimeUnit::SECONDS) {
            uint32_t endTime;
            if(mRunning) {
                endTime = millis();
            } else {
                endTime = mEndTime;
            }
            return GetTimeElapsed(endTime, mLastFrameTime);
        }
    
        double GetTotalElapsed(TimeUnit unit = TimeUnit::SECONDS) {
            uint32_t endTime;
            if(mRunning) {
                endTime = millis();
            } else {
                endTime = mEndTime;
            }
            return GetTimeElapsed(endTime, mStartTime);
        }
};