#include <limits>
#include <iostream>

using namespace std;
int main(){
	cout << "max float is: " << std::numeric_limits<float>::max() << "\n";
	cout << "min float is: " << std::numeric_limits<float>::min() << "\n";
	cout << "infinity is: " << std::numeric_limits<float>::infinity() << "\n";
}