%%创建随机解函数  得到的x和y是n维度向量
function sol1=CreateRandomSolution(model)

    n=model.n;
    
    xmin=model.xmin;
    xmax=model.xmax;
    
    ymin=model.ymin;
    ymax=model.ymax;

    %%生成1*n阶在【xmin，xmax】和在【ymin，ymax】内的向量
    sol1.x=unifrnd(xmin,xmax,1,n);
    sol1.y=unifrnd(ymin,ymax,1,n);
    
end