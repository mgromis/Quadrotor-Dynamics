function [ new_flow, new_A] = ransac(n, unsorted_flow, thresh, A_total, sorted_flow, on )
%RANSAC Summary of this function goes here
%   Detailed explanation goes here
if(on)
   max_inliers = 0;
   inl_index = []; 

   for i =1:500
       
       
       rand_indices = randperm(size(unsorted_flow,1),n);
       dots = unsorted_flow(rand_indices',:);
       test_flow = zeros(2*n,1); 
       A = zeros(2*n,6);
       for k = 1:n
       test_flow((2*k-1), :) = dots(k,1);
       test_flow((2*k), :) = dots(k,2);
       
       A((2*k-1), :) = A_total(2*rand_indices(k)-1,:);
       A((2*k), :) = A_total(2*rand_indices(k),:);
       end
       

    vels = A\test_flow;
    error = sorted_flow-A_total*vels;
    err_vec = [];
    for g = 1:(size(error,1)/2)
        err_vec = [err_vec; (error(2*g-1)^2 + error(2*g)^2)^.5];
        
    end
    
    possible_max_inl_index = find(err_vec<thresh);
    if(size(possible_max_inl_index,1)>max_inliers)
    inl_index = possible_max_inl_index;
    max_inliers = size(possible_max_inl_index,1);
    end
    
    end
    
    new_flow = zeros(2*size(inl_index,1),1);
    new_A = zeros(2*size(inl_index,1),6);
    for i = 1:size(inl_index,1)
        new_flow(2*i-1,:)  = sorted_flow(2*inl_index(i)-1,:);
        new_flow(2*i,:)  = sorted_flow(2*inl_index(i),:);
        new_A(2*i-1,:) = A_total(2*inl_index(i)-1,:);
        new_A(2*i,:)   = A_total(2*inl_index(i),:);
    end
else
    new_flow = sorted_flow;
    new_A = A_total;
    
end
end

   

   
   

