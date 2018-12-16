%% Simulation
system_size = 2*N + 4*regressors.size_g + 2*regressors.size_f;
state = zeros(system_size,1);
T_sim = [0,DT];
t_start = 0;

%% Store Results
T_total  = [0];                   % overall time vector
X_total  = zeros(1,system_size);  % overall states vector
UR_total = [];                    % reference plant control input

if load_sim
    % Load Previous simulation file
    load([sim_direction,'sim_loop_',num2str(loop_start-1),'_6']);
    T_total = T_total(end);
    X_total = X_total(end,:);
    state  = X_total(end,:)';
    UR_total = UR_total(end,:);
    
    % loops
    T_sim = [T_total(end),T_total(end)+DT];
    t_start = T_total(end);
    loop_vec = loop_start:loop_end;
else
    loop_vec = 1:loops;
end

%% Simulation Loop
for i = loop_vec
    
    for c = optimal_seq
        %% Calculated activated centers for this loop
        if c == 1 
            index_f = 1:regressors.size_f;
            index_g = 1:regressors.size_g;
        else
            index_f = active_centers{c-1,1}';
            index_g = 1:regressors.size_g;     % simulate full g
        end

        % Simulated regressor size and indexes
        regressors.size_f_sim = size(index_f,2);
        regressors.size_g_sim = size(index_g,2);
        regressors.index_f = index_f;
        regressors.index_g = index_g;
        
        % Initial states vector
        total_index = index_calculator(N,regressors);
        x0_sim = state(total_index);
        
        %% Next targets for reference models
        c0 = [state([5,6]);0];
        cT = [centers_f([1,2],c);0];
        ref_params.theta_1 = polynomial_ref_coeff(c0,cT,0,DT);

        c0 = [state([7,8]);0];
        cT = [centers_f([3,4],c);0];
        ref_params.theta_2 = polynomial_ref_coeff(c0,cT,0,DT);
        
        % wf and wg hat
        weights.wf_hat = state(9:(8+2*regressors.size_f));
        weights.wg_hat = state((1+end-4*regressors.size_g):end);

        %% ODE Invocation
        t_start = T_sim(1);
        sim_fun = @ (t,x) robotic_2dof_idnt( t,x,ref_params,ppc_fun,regressors,gains,weights,sim_plant,t_start );
        [T,X] = ode15s(sim_fun,T_sim,x0_sim);
        
        %% Update states
        state(total_index) = X(end,:);
        
        % Append Solutions
        % Time
        T_total = [T_total;T];
        
        % Solution Matrix
        X_append = zeros(size(T,1),system_size);
        X_append(:,total_index) = X;       
        not_idx = not(ismember(1:system_size,total_index));
        X_append(:,not_idx) = repmat(X_total(end,not_idx),size(T,1),1);  
        X_total = [X_total;X_append];
        
        % reference input
        UR1 = ref_control_input_poly(ref_params.theta_1,T-t_start);
        UR2 = ref_control_input_poly(ref_params.theta_2,T-t_start);
        UR_total = [UR_total;UR1,UR2];
        
        % Update time vector
        T_sim = [T(end) T(end)+DT];
        
        %% Reference model update for next iteration
        ref_params.theta_1.x0 = state(5:6);
        ref_params.theta_2.x0 = state(7:8);
        
        ref_params.theta_1.t_start = T(end);
        ref_params.theta_2.t_start = T(end);
        disp(['Center = ',num2str(c),' || Loop = ',num2str(i),' || index = ',num2str(c)]);    
        
    end

    %%% Save results and clear memory
    disp('Saving')
    if save_results == true
        save([results_dir,'sim_loop_',num2str(i)],...
              'X_total','T_total','UR_total','optimal_seq','active_centers');
    end
    clear('T_total','X_total','UR_total');

    % Initialization for next loop
    T_total = T(end);
    X_total = state';
    UR_total = [UR1(end),UR2(end)];

end
