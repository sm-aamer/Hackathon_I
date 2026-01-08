import React from 'react';
import clsx from 'clsx';
import styles from './SimulationExample.module.css';

const SimulationExample = ({title, children, type = 'general'}) => {
  const simulationTypeClass = type ? `simulation-${type}` : '';

  return (
    <div className={clsx('simulation-example-container', styles.container, simulationTypeClass)}>
      <div className="simulation-example-header">
        <h3>{title}</h3>
      </div>
      <div className="simulation-example-content">
        {children}
      </div>
    </div>
  );
};

export default SimulationExample;