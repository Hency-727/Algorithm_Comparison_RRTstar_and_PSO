%%环境建模函数
function model=CreateModel(start,goal,obs)

    % Source
    xs=start(1);
    ys=start(2);
    
    % Target (Destination)
    xt=goal(1);
    yt=goal(2);
    
    xobs=obs(:,1);
    yobs=obs(:,2);
    robs=obs(:,3);
    
    n=3;
    
    xmin=-10;
    xmax= 10;
    
    ymin=-10;
    ymax= 10;
    
    model.xs=xs;
    model.ys=ys;
    model.xt=xt;
    model.yt=yt;
    model.xobs=xobs;
    model.yobs=yobs;
    model.robs=robs;
    model.n=n;
    model.xmin=xmin;
    model.xmax=xmax;
    model.ymin=ymin;
    model.ymax=ymax;
    
end