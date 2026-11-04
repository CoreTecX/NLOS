struct RInverse {
    double ch[H][H],cw[W][W];Vec t,b;
    RInverse():t(S),b(S){double pi=std::acos(-1.);
        for(int k=0;k<H;++k)for(int j=0;j<H;++j)ch[k][j]=std::sqrt((k?2.:1.)/H)*std::cos(pi*(j+.5)*k/H);
        for(int k=0;k<W;++k)for(int j=0;j<W;++j)cw[k][j]=std::sqrt((k?2.:1.)/W)*std::cos(pi*(j+.5)*k/W);
    }
    void run(const Vec& a,const Vec& spec,Vec& y){
        for(int k=0;k<H;++k)for(int j=0;j<W;++j)for(int c=0;c<3;++c){double s=0;for(int r=0;r<H;++r)s+=ch[k][r]*a[(r*W+j)*3+c];t[(k*W+j)*3+c]=s;}
        for(int k=0;k<H;++k)for(int j=0;j<W;++j)for(int c=0;c<3;++c){double s=0;for(int r=0;r<W;++r)s+=cw[j][r]*t[(k*W+r)*3+c];b[(k*W+j)*3+c]=s/spec[k*W+j];}
        for(int k=0;k<H;++k)for(int j=0;j<W;++j)for(int c=0;c<3;++c){double s=0;for(int r=0;r<H;++r)s+=ch[r][k]*b[(r*W+j)*3+c];t[(k*W+j)*3+c]=s;}
        for(int k=0;k<H;++k)for(int j=0;j<W;++j)for(int c=0;c<3;++c){double s=0;for(int r=0;r<W;++r)s+=cw[r][j]*t[(k*W+r)*3+c];y[(k*W+j)*3+c]=s;}
    }
};
