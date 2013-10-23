//appends a list to a list
//argument0 - list
//Argument1 - additional list
//returns list id
var i,list;
for(i=0;i<ds_list_size(argument1);i+=1){
   ds_list_add(argument0,ds_list_find_value(argument1,i));
}
return(argument0);
