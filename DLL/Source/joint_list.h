#include "dll.h"

struct node;

struct dlist;

double ds_list_size(dlist *list);

double ds_list_add(dlist *list, double value);

double ds_list_delete(dlist *list, double pos);

double ds_list_find_value(dlist *list, double pos);

/*
export double gmn_list_create();

export double gmn_list_add(double blist, double value);

export double gmn_list_size(double blist);
*/
