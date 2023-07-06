 [xData, yData] = prepareCurveData( x, y );
    % 设置 fittype 和选项。
    ft = fittype( 'a*x-(9.8/(2*b^2))*x^2', 'independent', 'x', 'dependent', 'y' );
    opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
    opts.Display = 'Off';
    opts.StartPoint = [0.498364051982143 0.959743958516081];
    
    % 对数据进行模型拟合。
    [fitresult, gof] = fit( xData, yData, ft, opts );
     
    hLines = plot(x,y,'o');
    title('轨迹');
    xlabel('水平位置(cm)');ylabel('竖直位置(cm)')
    grid('on')
    hold('on')
    warnState = warning('off','MATLAB:fplot:NotVectorized');
  
    hLines(2) = plot(xData, yData);
    warning(warnState)
    hLines(2).LineWidth = 2;
    hold('off')
    legend(msg('LegendMeasuredData'),msg('LegendTheoretical'),'Location','Northeast')
    
