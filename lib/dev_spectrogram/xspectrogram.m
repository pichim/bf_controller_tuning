function qmesh = xspectrogram(x, y, Ts, Nest, resolution, vmin, vmax, ax)

    % check for input arguments and set defaults if not provided
    if nargin < 4 || isempty(Nest)
        Nest = 200;
    end
    if nargin < 5 || isempty(resolution)
        resolution = 200;
    end
    if nargin < 6 || isempty(vmin)
        vmin = 1e-1;
    end
    if nargin < 7 || isempty(vmax)
        vmax = 1e1;
    end
    if nargin < 8 || isempty(ax)
        ax = gca; % use current axes if none provided
    end

    % Todo: check if it is usefull to remove mean here
    % x = x - mean(x);
    % y = y - mean(y);

    Ndata = size(x, 1);

    % calculate overlap size for windowing
    Noverlap = floor(0.9 * Nest);

    % find the minimum and maximum values of y and generate linearly spaced vector for the output y-axis
    ymin = min(y);
    ymax = max(y);
    dy = (ymax - ymin) / resolution;
    y_axis = (ymin:dy:ymax-dy).';

    % compute the frequency bins for the output x-axis
    % freq = (0:Nest/2).' * (1 / (Nest * Ts));
    df = 1/(Nest*Ts);
    freq = (0:df:1/Ts-df).';

    % initialize matrices for the spectrogram data and counter for averaging
    Pavg = zeros(resolution, Nest);
    Navg = zeros(resolution, 1);

    % generate the windowing function
    window = hann(Nest);

    % factor 2 so that the magnitude corresponds to a single sided spectrum
    % 2.3*sin(2*pi*f0*time) <=> sqrt(puu(f0)) = 2.3
    W = sum(window) / Nest / 2;

    ind_start = 1;
    ind_end = Nest;
    Ndelta  = Nest - Noverlap;

    % process the signal in chunks
    while ind_end <= Ndata

        % calculate the start and end indices for the y values
        y_ind_start = round(ind_start + (Nest - Ndelta)/2);
        y_ind_end   = round(ind_start + (Nest + Ndelta)/2);

        % map y values to indices in the spectrogram resolution
        indices = round((y(y_ind_start:y_ind_end) - ymin) / (ymax - ymin) * (resolution - 1)) + 1;

        ind = ind_start:ind_end;

        inp_act = x(ind);

        inp_act = window .* inp_act;

        % Todo: check if it is usefull to remove mean here
        % inp_act = inp_act - mean(inp_act);

        U = fft(inp_act) / (Nest * W);
        Pact = U .* conj(U);

        % accumulate the magnitudes into the spectrogram matrix
        for i = 1:length(indices)
            idx = indices(i);
            Pavg(idx, :) = Pavg(idx, :) + Pact.';
            Navg(idx) = Navg(idx) + 1;
        end

        ind_start = ind_start + Ndelta;
        ind_end   = ind_end   + Ndelta;
    end


    ind_Navg = Navg ~= 0;
    Pavg = Pavg(ind_Navg,:) ./ Navg(ind_Navg);

    ind = freq <= 1 / (2 * Ts);
    freq = freq(ind);
    Pavg = Pavg(:, ind);

    % Plot the spectrogram data as a pseudocolor plot
    qmesh = pcolor(ax, freq, y_axis, sqrt(Pavg));
    set(qmesh, 'EdgeColor', 'none'); % Remove gridlines
    clim(ax, [vmin vmax]); % Set the color axis scaling
    c = colorbar(ax); % Add a colorbar
    set(ax, 'ColorScale', 'log');
    xlabel(ax, 'Frequency (Hz)'); % Label the y-axis
end
