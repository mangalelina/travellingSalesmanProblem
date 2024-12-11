#include <iostream>
#include "drone.hpp"
#include <iomanip>
#include "xcode_redirect.hpp"

using namespace std;

int main(int argc, char * argv[]) {
    xcode_redirect(argc, argv);
    std::ios_base::sync_with_stdio(false);
    cout << std::setprecision(2); //Always show 2 decimal places
    cout << std::fixed; //Disable scientific notation for large numbers
    cerr << fixed << showpoint << setprecision(2) << boolalpha;
    Drone(argc, argv);
    return 0;
}
