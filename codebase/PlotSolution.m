%pso的起点、终点、轨迹
function PlotSolution(sol,model)

    xs=model.xs;
    ys=model.ys;
    xt=model.xt;
    yt=model.yt;
    xobs=model.xobs;
    yobs=model.yobs;
    robs=model.robs;
    
    XS=sol.XS;
    YS=sol.YS;
    xx=sol.xx;
    yy=sol.yy;
    
    theta=linspace(0,2*pi,100);
%     for k=1:numel(xobs)
%         fill(xobs(k)+robs(k)*cos(theta),yobs(k)+robs(k)*sin(theta),[0.5 0.7 0.8]);
%         hold on;
%     end

     plot(xx,yy,'r','LineWidth',2);  %pso轨迹  r
     plot(XS,YS,'ro');          %过程圈点轨迹
     plot(xs,ys,'rh','MarkerSize',16,'MarkerFaceColor','y');    %起点
     plot(xt,yt,'g^','MarkerSize',16,'MarkerFaceColor','c');    %终点
    grid on;
    axis equal;

end