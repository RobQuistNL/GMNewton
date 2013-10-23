#include <cstddef>
#include <stdafx.h>
#include "newton.h"
#include "dll.h"
#include "recast.h"



class Joint_List{//Class delaration
    public:
        Joint_List(int array_size);
        ~Joint_List();
        int CountElements();
        void ChangeSize(int array_size);
        void AddElement(int value);
        void RemoveElement(int pos);
        int FindElementValue(int pos);
        int FindElementPos(int value);
    private:
        int *the_array;
        int size;
        int elements;
};

Joint_List::Joint_List(int array_size){//Constructor
    the_array = new int[array_size];
    size = array_size;
    elements = 0;
}

Joint_List::~Joint_List(){//Destructor
    delete[ ] the_array;
}

int Joint_List::CountElements(){//Returns the number of elements in list
    return(elements);
}

void Joint_List::ChangeSize(int array_size){//Changes the size of the list
    int *temp = new int[array_size];
    for(int i=0; i<array_size; i++){
        temp[i]=the_array[i];
    }
    delete[ ] the_array;
    the_array = temp;
    size = array_size;
    if(array_size<elements){
        elements = array_size;
    }
}

void Joint_List::AddElement(int value){//Adds an element to list
    if(elements>=size){
        ChangeSize(size+10);
    }
    the_array[elements] = value;
    elements+=1;
}

void Joint_List::RemoveElement(int pos){//Removes an element from list
    if((pos<elements)&&(pos>=0)){//Make sure element exists
        int *temp = new int[size];
        for(int i=0; i<pos; i++){
            temp[i]=the_array[i];
        }

        for(int i=pos+1; i<elements; i+=1){
            temp[i-1]=the_array[i];
        }

        delete[ ] the_array;
        the_array = temp;
        elements -= 1;
    }
}

int Joint_List::FindElementValue(int pos){//finds value at position
    if(pos<elements){//Make sure element exists
        return(the_array[pos]);
    }else{
        return(-1);
    }
}

int Joint_List::FindElementPos(int value){//finds first position with value
    for(int i=0; i<elements; i++){//Cycle through array
        if(the_array[i]==value){//Check if array[i] matches value
            return(i);//Return position if match found
        }
    }
    return(-1);//No positions held value
}
