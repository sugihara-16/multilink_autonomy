#include "synchronized_images.hpp"
#include <cassert>
#include <iostream>
int main(){
 SynchronizedImages<int> q(3,2);
 assert(!q.push(2,100,30));assert(!q.push(0,100,10));assert(!q.push(0,100,99));
 auto b=q.push(1,100,20);assert(b && b->stamp==100 && b->images==std::vector<int>({10,20,30}));assert(q.duplicates==1);
 assert(!q.push(1,100,40) && q.late==1);
 assert(!q.push(0,101,1));assert(!q.push(1,102,2));assert(!q.push(2,102,3));assert(!q.push(1,103,4));assert(q.dropped==1 && q.pending()==2);
 b=q.push(0,102,1);assert(b && b->stamp==102);assert(!q.push(2,101,9));
 assert(!q.push(0,104,10));assert(!q.push(1,104,20));b=q.push(2,104,30);assert(b && q.dropped==2 && q.pending()==0);
 bool caught=false;try{q.push(3,200,1);}catch(const std::out_of_range&){caught=true;}assert(caught);
 SynchronizedImages<int> one(1,1);assert(one.push(0,1,0)->images[0]==0);
 std::cout<<"exact timestamps, reordered arrivals, duplicates, missing frames, bounded queue, late frames, single camera: PASS\n";
}
