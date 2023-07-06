function HSVpie(hImProc, hAxHSV, HSV_Thresh)
%%
n = 360;
r = 1;

im = hImProc.CData;
im_hsv = rgb2hsv(im);

fh = ancestor(hAxHSV,'figure');
fh.WindowButtonUpFcn = @unclickFcn;
hAxHSV.ButtonDownFcn = @clickAxesFcn;

x1 = r*cos(linspace(0,2*pi,n+1));
y1 = r*sin(linspace(0,2*pi,n+1));
x = [zeros(1,n);x1(1:end-1);x1(2:end);zeros(1,n)];
y = [zeros(1,n);y1(1:end-1);y1(2:end);zeros(1,n)];
colormap(fh,hsv(n));
patch(hAxHSV,x,y,'r','FaceColor','flat','CData',(0:n-1)','EdgeColor','none','HitTest','off')
axis(hAxHSV,'equal')
axis(hAxHSV,'off')

th = 0:360;

minValDeg = HSV_Thresh(1)*360;
maxValDeg = HSV_Thresh(2)*360;

if minValDeg <= maxValDeg
    id = th >= minValDeg & th <= maxValDeg;
else
    id = [find(th >= minValDeg) find(th <= maxValDeg)];
end

xx = r*cosd([minValDeg th(id) maxValDeg]);
yy = r*sind([minValDeg th(id) maxValDeg]);

h0 = line(xx,yy,'Parent',hAxHSV,'LineWidth',2,'Color','k');

h1 = line([0 r*cosd(minValDeg)],[0 r*sind(minValDeg)],'Parent',hAxHSV,'Color','k','LineWidth',2,'ButtonDownFcn',@clickMinLine);
h2 = line([0 r*cosd(maxValDeg)],[0 r*sind(maxValDeg)],'Parent',hAxHSV,'Color','k','LineWidth',2,'ButtonDownFcn',@clickMaxLine);

[~,im2] = createMask_HSV(im,im_hsv,minValDeg/360,maxValDeg/360);
hImProc.CData = im2;


    function clickAxesFcn(varargin)
        pt = hAxHSV.CurrentPoint;
        in = inpolygon(pt(1,1),pt(1,2),[0 xx 0],[0 yy 0]);
        if in
            deg = atan2d(pt(1,2),pt(1,1));
            %toMin = 
        end
    end

    function clickMinLine(varargin)
        fh.WindowButtonMotionFcn = @(o,e) dragline(h1);
    end

    function clickMaxLine(varargin)
        fh.WindowButtonMotionFcn = @(o,e) dragline(h2);
    end

    function unclickFcn(varargin)
        fh.WindowButtonMotionFcn = '';
        %fprintf('[%f  %f]\n',minValDeg/360,maxValDeg/360)
    end

    function dragline(h)
        pt = hAxHSV.CurrentPoint;
        deg = atan2d(pt(1,2),pt(1,1));
        if deg < 0
            deg = deg+360;
        end
        if isequal(h, h1)
            minValDeg = deg;
        else
            maxValDeg = deg;
        end
        
        [xx, yy] = calcRangeData(minValDeg,maxValDeg);
        
        set(h0,'XData',xx,'YData',yy)
        
        if isequal(h,h1)
            set(h1,'XData',[0 r*cosd(minValDeg)],'YData',[0 r*sind(minValDeg)])
        else
            set(h2,'XData',[0 r*cosd(maxValDeg)],'YData',[0 r*sind(maxValDeg)])
        end
        
        fh.UserData = [minValDeg/360 maxValDeg/360];
        
        [~,im2] = createMask_HSV(im,im_hsv,minValDeg/360,maxValDeg/360);
        hImProc.CData = im2;
    end

    function [xx, yy] = calcRangeData(minValDeg,maxValDeg)
        if minValDeg <= maxValDeg
            id = th >= minValDeg & th <= maxValDeg;
        else
            id = [find(th >= minValDeg) find(th <= maxValDeg)];
        end
        
        xx = r*cosd([minValDeg th(id) maxValDeg]);
        yy = r*sind([minValDeg th(id) maxValDeg]);
    end

end


