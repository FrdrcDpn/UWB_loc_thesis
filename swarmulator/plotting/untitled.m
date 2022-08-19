% plot our samples (only for 2D case)
scatter(X(:,1), X(:,2), 'filled'), hold on
ezcontour(@(x,y) mvnpdf([x y], mu, sigma), xlim(), ylim())




   % title('X~N(\mu,\sigma)')
   % xlabel('X_1'), ylabel('X_2')