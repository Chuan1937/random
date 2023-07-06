function [BW, img_out] = createMask_RGB(img_in,thresh)



R = img_in(:,:,1);
G = img_in(:,:,2);
B = img_in(:,:,3);

BW = R >= thresh(1) & R <= thresh(2) & ...
    G >= thresh(3) & G <= thresh(4) & ...
    B >= thresh(5) & B <= thresh(6);

if nargout == 2
    R(~BW) = 0;
    G(~BW) = 0;
    B(~BW) = 0;
    
    img_out = cat(3,R,G,B);
end
