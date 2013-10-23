#include "joint_list.h"

#ifndef NULL
#define NULL 0
#endif


struct node{
    double joint;
    node *nxt;
};

struct dlist{
    node *front;
    node *back;

    dlist(){
        front = NULL;
        back = NULL;
    }
};

double ds_list_size(dlist *list){
    //Return size of list
    node *temp = list->front;
    int count=0;

    while(temp->nxt!=0){
        count+=1;
        temp = temp->nxt;
    }
}

double ds_list_add(dlist *list, double value){
    //Add a value(joint) to the end of the list
    node *newptr, *temp,*temp2; //temporary pointers

    newptr = new node;    //Create a new node
    newptr->joint = value;
    newptr->nxt = NULL;

    if(list->front == 0){//If there are no nodes in list, this is the first one
        list->front = newptr;
        list->back = newptr;
    }else{//otherwise arrange pointers accordingly to accomedate new node
        temp2=list->front;
        while(temp2->nxt != 0){
            temp2 = temp2->nxt;
        }
        temp2->nxt = newptr;
        list->back=newptr;
    }
}

double ds_list_delete(dlist *list, double pos){
    node *temp, *temp2;
    temp = list->front;

    if(list->front == list->back){//If front and back nodes are the same (either NULL or a node)
        if(list->front != 0){//If there is only one node
            delete list->front;//Delete it
            list->front = NULL;
            list->back = NULL;
            return (1);
        }else{//Otherwise if there are no nodes,
            return(0);//Do nothing
        }
    }
    else if(pos<ds_list_size(list)){//If there are multiple nodes

        for(int i=0; i<pos; i+=1){//Find correct node and one behind it
            temp2 = temp;
            temp = temp->nxt;
        }

        if(list->front == temp){//If node is the first one
            list->front = temp->nxt;
            delete temp;
            return (1);
        }else if(list->back == temp){//If node is last one
            list->back = temp2;
            temp2->nxt = NULL;
            delete temp;
            return(1);
        }else{//If node is somewhere in middle
            temp2->nxt = temp->nxt;
            delete temp;
            return (1);
        }

    }
    return (0);//Return 0 if invalid 'pos' passed
}

double ds_list_find_value(dlist *list, double pos){
    node *temp;
    temp = list->front;
    if(pos<ds_list_size(list)){
        for(int i=0; i<pos; i+=1){
            temp = temp->nxt;
        }
        return(temp->joint);
    }
    return (-1);
}

double ds_list_add_old(dlist *list, double value){
    //Add a value(joint) to the list
    node *temp,*temp2; //temporary pointers

    temp = new node;
    temp->joint = value;
    if(list->front->nxt == 0){
        list->front = temp;
    }else{
        temp2=list->front;
        while(temp2->nxt != 0){
            temp2 = temp2->nxt;
        }
    }
}

/*
export double gmn_list_create(){
    dlist *list = new dlist;
    return ( (double)reinterpret_cast<int>(list) );
}

export double gmn_list_add(double blist, double value){
   dlist *list = reinterpret_cast<dlist*>((int)blist);
   return( ds_list_add(list, value) );
}

export double gmn_list_size(double blist){
    dlist *list = reinterpret_cast<dlist*>((int)blist);
    return( ds_list_size(list) );
}
*/
