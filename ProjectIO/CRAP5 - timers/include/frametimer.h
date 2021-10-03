#include <chrono>

// Unit of time to measure by
enum TimeUnit {
    SECONDS = 0,
    MILLISECONDS = 1,
    MICROSECONDS = 2,
    NANOSECONDS = 3
};

// Set of enums for precision of time
// A.k.a number of decimal places
enum TimePrecision {
    DP0 = 0,
    DP3 = 1,
    DP6 = 2,
    DP9 = 3
};

class FrameTimer {

    typedef std::chrono::high_resolution_clock time_clock;
    typedef std::chrono::microseconds time_unit;

    private:
        std::chrono::time_point<time_clock> mStartTime;
        std::chrono::time_point<time_clock> mEndTime;
        std::chrono::time_point<time_clock> mLastFrameTime;
        std::chrono::time_point<time_clock> mLastLastFrameTime;
        bool mRunning = true;

        // Gets the elapsed seconds time difference between t1 and t2 (t1 - t2)
        double GetElapsedSeconds(std::chrono::time_point<time_clock> t1, std::chrono::time_point<time_clock> t2) {
            return std::chrono::duration_cast<std::chrono::seconds>(t1 - t2).count();
        }

        // Gets the elapsed milliseconds time difference between t1 and t2 (t1 - t2)
        double GetElapsedMilliseconds(std::chrono::time_point<time_clock> t1, std::chrono::time_point<time_clock> t2) {
            return std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t2).count();
        }

        // Gets the elapsed microseconds time difference between t1 and t2 (t1 - t2)
        double GetElapsedMicroseconds(std::chrono::time_point<time_clock> t1, std::chrono::time_point<time_clock> t2) {
            return std::chrono::duration_cast<std::chrono::microseconds>(t1 - t2).count();
        }

        // Gets the elapsed nanoseconds time difference between t1 and t2 (t1 - t2)
        double GetElapsedNanoseconds(std::chrono::time_point<time_clock> t1, std::chrono::time_point<time_clock> t2) {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t2).count();
        }

        double GetTimeElapsed(   std::chrono::time_point<time_clock> t1,
                                    std::chrono::time_point<time_clock> t2,
                                    TimeUnit unit = TimeUnit::SECONDS,
                                    TimePrecision precision = TimePrecision::DP3) {
            int readUnit = unit + precision > TimeUnit::NANOSECONDS ? TimeUnit::NANOSECONDS : unit + precision;
            int readPrecision = readUnit - unit;
            int readScalar = pow(10, 3 * readPrecision);
            long long elapsedTime;
            switch (readUnit) {
            case TimeUnit::SECONDS:
                elapsedTime = GetElapsedSeconds(t1, t2);
                break;
            case TimeUnit::MILLISECONDS:
                elapsedTime = GetElapsedMilliseconds(t1, t2);
                break;
            case TimeUnit::MICROSECONDS:
                elapsedTime = GetElapsedMicroseconds(t1, t2);
                break;
            case TimeUnit::NANOSECONDS:
                elapsedTime = GetElapsedNanoseconds(t1, t2);
                break;
            
            default:
                std::cout << "ERROR: Time unit and precision broken" << std::endl;
                return 0;
            }
            return (double)elapsedTime / (double)readScalar;
        }


    public: 
        FrameTimer() {
            Start();
            Frame();
        }

        void Start() {
            mStartTime = time_clock::now();
            mLastFrameTime = mStartTime;
            mLastLastFrameTime = mStartTime;
            mRunning = true;
        }

        void Stop() {
            mEndTime = time_clock::now();
            mRunning = false;
        }
        
        void Frame() {
            if (mRunning) {
                mLastLastFrameTime = mLastFrameTime;
                mLastFrameTime = time_clock::now(); 
            }
        }

        double GetLastFrameElapsed(TimeUnit unit = TimeUnit::SECONDS, TimePrecision precision = TimePrecision::DP3) {
            return GetTimeElapsed(mLastFrameTime, mLastLastFrameTime);
        }

        double GetCurrentFrameElapsed(TimeUnit unit = TimeUnit::SECONDS, TimePrecision precision = TimePrecision::DP3) {
            std::chrono::time_point<time_clock> endTime;
            if(mRunning) {
                endTime = time_clock::now();
            } else {
                endTime = mEndTime;
            }
            return GetTimeElapsed(endTime, mLastFrameTime);
        }
    
        double GetTotalElapsed(TimeUnit unit = TimeUnit::SECONDS, TimePrecision precision = TimePrecision::DP3) {
            std::chrono::time_point<time_clock> endTime;
            if(mRunning) {
                endTime = time_clock::now();
            } else {
                endTime = mEndTime;
            }
            return GetTimeElapsed(endTime, mStartTime);
        }
};