#include <iostream>
using namespace std;

void handleValue(int i) { cout << i << ", "; }
void processValues() { }

template<typename T1, typename... Tn>
void processValues(T1 arg1, Tn... args)
{
    handleValue(arg1);
    processValues(args...);
}

int main()
{
    processValues(1);
    
    return 0;
}
