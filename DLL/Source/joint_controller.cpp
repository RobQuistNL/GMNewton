#include "joint_controller.h"

double add_joint_to_list(NewtonJoint list[], int max, int size, NewtonJoint nJoint){
    //int max = 10;           // maximum size of array
    //int size = 0;           // current size of array
    //NewtonJoint* list = new NewtonJoint[max];

    list[size] = nJoint;
    size++;
    if (size >= max) {
        max = max * 2;            // double the previous size
        NewtonJoint* temp = new NewtonJoint[max]; // create new bigger array.
        for (int i=0; i<size; i++) {
            temp[i] = list[i];       // copy values to new array.
        }
        delete [] list;              // free old array memory.
        list = temp;                 // now a points to new array.
    }
}








/*
int max = 10;           // no longer const
int* a = new int[max];  // allocated on heap
int n = 0;

//--- Read into the array
while (cin >> a[n]) {
    n++;
    if (n >= max) {
        max = max * 2;            // double the previous size
        int* temp = new int[max]; // create new bigger array.
        for (int i=0; i<n; i++) {
            temp[i] = a[i];       // copy values to new array.
        }
        delete [] a;              // free old array memory.
        a = temp;                 // now a points to new array.
    }
}
//--- Write out the array etc.
*/

/*
int* pos = (int*) NewtonBodyGetUserData(body);
   force = force + add_forces[*pos];


    int* pos=new int(body_list.size()-1);//Get virtual id of world
   NewtonBodySetUserData(nBody,pos);//Store the virtual id of the body
   return (double)(*pos);//Return virtual id of body

*/
