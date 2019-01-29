figure(1)
hold off;

t = state.time;
sv = state.signals.values;
rv = ref.signals.values;

a = 1;
%b = [1/8 1/8 1/8 1/8 1/8 1/8 1/8 1/8];
b = [1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16];
y = filter(b,a,sv);

plot(t,y,t,rv,'--' , 'LineWidth' , 1.2); %choose between filtered and
                                          %unfiltered data
%plot(t,sv,t,rv,'--', 'LineWidth', 1.2);

legend('Elev rate','Elev rate reference') % Up rigth corner legends
handles(1) = xlabel('$t$'); % xLabel
handles(2) = ylabel('$\dot{\tilde{e}}$'); %yLabel
set(handles, 'Interpreter' , 'Latex'); % Making them in latex
set(handles, 'Fontsize' , 20); % Fontsize
set(get(gca,'ylabel'),'rotation',0) % % Rotates text on ylabel

set(gcf, 'PaperPositionMode', 'auto');
print -depsc2 p3p3_elev_rate.eps %Sets the filename for export
%close;




