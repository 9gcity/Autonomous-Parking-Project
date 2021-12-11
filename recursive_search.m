function i = recursive_search(in, array)
    num_arr = numel(array);
    half = round(num_arr/2);
    if in == array(half)
        i = array(half);
    elseif num_arr == 2
        if in > ((array(2) - array(1))/2)
            i = array(2);
        else
            i = array(1);
        end
    elseif in < array(half)
        i = recursive_search(in, array(1:half));
    else
        i = recursive_search(in, array(half:end));
    end
end