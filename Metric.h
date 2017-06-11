template <class T>
class Metric
{

    uint32_t _count;
    const char* _name;
    T _min;
    T _max;
    T _sum;
    uint32_t _printInterval;
public:
    Metric(const char* name) {
        _name=name;
        _printInterval=1000000;
    };
    Metric(const char* name,uint32_t printInterval) {
        _name=name;
        _printInterval=printInterval;
    };
    void update(T value) {
        if ( _count==0 ) {
            _min=_max=value;
            _sum=value;
            _count=1;
            return;
        }
        if ( _min > value ) _min=value;
        if ( _max < value ) _max = value;
        _sum+=value;
        _count++;
        if ( _printInterval == _count ) {
            print();
            reset();
        }
    };
    uint32_t count() {
        return _count;
    }
    void reset() {
        _count=0;
    }
    void print() {
        INFO(" %s - min : %d , max : %d , avg : %d , count : %d ",_name, _min,_max,_sum/_count,_count);
    }
};

class Timer : public Metric<uint32_t>
{
    uint64_t _startTime;
public :
    Timer(const char* name,uint32_t counter) : Metric(name,counter) {

    }
    void start() {
        _startTime=micros();
    }
    void stop() {
        update(micros()-_startTime);
    }
};
